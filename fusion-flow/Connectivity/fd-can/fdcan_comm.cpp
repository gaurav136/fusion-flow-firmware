/*
 * fdcan_comm.cpp
 *
 * FDCAN communication library implementation for STM32G4
 * Improved version focusing on safety and encapsulation
 *
 * Created on: Dec 9, 2025
 * Author: dark
 */

#include "fdcan_comm.hpp"
FDCANCommunication g_fdcan_comm(&hfdcan1); // one true definition

/* ============================================================================
 * Constructor
 * ============================================================================ */

FDCANCommunication::FDCANCommunication(FDCAN_HandleTypeDef* handle)
  : hfdcan(handle),
    rx_callback(nullptr),
    error_callback(nullptr),
    is_initialized(false),
    is_bus_off(false)
{
    // Ensure the handle is valid before proceeding
    if (hfdcan == nullptr) {
        // Log error or set a flag if logging/error reporting is available
    }
    std::memset(&tx_header_cache, 0, sizeof(tx_header_cache));
    std::memset(&active_config, 0, sizeof(active_config));
}

/* ============================================================================
 * Initialization
 * ============================================================================ */

bool FDCANCommunication::init(const FDCANConfig& config, FDCANRxCallback rx_cb, FDCANErrorCallback error_cb)
{
    if (hfdcan == nullptr) return false;

    /* Validate inputs */
    if (rx_cb == nullptr || error_cb == nullptr) {
        return false;
    }

    /* Store callbacks and configuration */
    rx_callback = rx_cb;
    error_callback = error_cb;
    active_config = config;

    /* 1. Configure Filter */
    if (!configRxFilter(config)) {
        return false;
    }

    /* 2. Configure TX Header Cache */
    configTxHeader(config);

    /* 3. Enable Comprehensive Interrupts */
    // Note: FDCAN_IT_RX_FIFO0_NEW_MESSAGE is essential for RX.
    // Others improve error reporting.
    if (HAL_FDCAN_ActivateNotification(hfdcan,
                                       FDCAN_IT_RX_FIFO0_NEW_MESSAGE |
                                       FDCAN_IT_ERROR_PASSIVE     |  // correct constant name
                                       FDCAN_IT_BUS_OFF           |
                                       FDCAN_IT_ARB_PROTOCOL_ERROR|
                                       FDCAN_IT_DATA_PROTOCOL_ERROR|
                                       FDCAN_IT_RX_FIFO0_MESSAGE_LOST,
                                       0U) != HAL_OK) {
        return false;
    }

    /* 4. Start FDCAN */
    if (HAL_FDCAN_Start(hfdcan) != HAL_OK) {
        return false;
    }

    is_initialized = true;
    is_bus_off = false;

    return true;
}

/* ============================================================================
 * Deinitialization
 * ============================================================================ */

bool FDCANCommunication::deinit(void)
{
    if (!is_initialized || hfdcan == nullptr) {
        return true; // Already deinitialized or invalid handle
    }

    /* Deactivate all active notifications */
    /* Deactivate specific notifications (fixed syntax) */
    if (HAL_FDCAN_DeactivateNotification(hfdcan,
                                     FDCAN_IT_RX_FIFO0_NEW_MESSAGE |
                                     FDCAN_IT_BUS_OFF |
                                     FDCAN_IT_ARB_PROTOCOL_ERROR |
                                     FDCAN_IT_DATA_PROTOCOL_ERROR |
                                     FDCAN_IT_RX_FIFO0_MESSAGE_LOST) != HAL_OK) {
        // Log error, but continue attempting to stop
    }

    /* Stop FDCAN */
    if (HAL_FDCAN_Stop(hfdcan) != HAL_OK) {
        return false;
    }

    is_initialized = false;
    return true;
}

/*=============================================================================
 * Configuration Helpers (Private)
 ============================================================================== */

bool FDCANCommunication::configRxFilter(const FDCANConfig& config)
{
    FDCAN_FilterTypeDef filter_config = {0};

    /* Configure RX Filter */
    filter_config.IdType = FDCAN_STANDARD_ID;
    filter_config.FilterIndex = 0;
    filter_config.FilterType = config.filter_type;
    filter_config.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    filter_config.FilterID1 = config.filter_id1;
    filter_config.FilterID2 = config.filter_id2;

    if (HAL_FDCAN_ConfigFilter(hfdcan, &filter_config) != HAL_OK) {
        return false;
    }

    /* Configure Global Filter: reject non-matching frames */
    if (HAL_FDCAN_ConfigGlobalFilter(hfdcan,
                                      FDCAN_REJECT, // Reject non-matching standard frames
                                      FDCAN_REJECT, // Reject non-matching extended frames
                                      FDCAN_FILTER_REMOTE, // Filter remote frames from standard
                                      FDCAN_FILTER_REMOTE) != HAL_OK) { // Filter remote frames from extended
        return false;
    }

    return true;
}

void FDCANCommunication::configTxHeader(const FDCANConfig& config)
{
    /* Cache static TX header fields. */
    tx_header_cache = (FDCAN_TxHeaderTypeDef){0};
    tx_header_cache.IdType = FDCAN_STANDARD_ID;
    tx_header_cache.TxFrameType = FDCAN_DATA_FRAME;
    tx_header_cache.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx_header_cache.BitRateSwitch = config.brs_on ? FDCAN_BRS_ON : FDCAN_BRS_OFF;
    tx_header_cache.FDFormat = FDCAN_FD_CAN;
    tx_header_cache.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    tx_header_cache.MessageMarker = 0U;
    // Identifier and DataLength are set per-message in transmit()
}

/* ============================================================================
 * Transmission (Thread-Safe)
 * ============================================================================ */

bool FDCANCommunication::transmit(uint32_t id, const uint8_t* data, uint32_t length)
{
    if (!is_initialized || data == nullptr || hfdcan == nullptr) {
        return false;
    }

    /* Create local copy of cached header to avoid race conditions
     * if transmit() is called concurrently from multiple contexts (e.g., main loop and ISR).
     */
    FDCAN_TxHeaderTypeDef tx_header_local = tx_header_cache;

    /* Update per-message fields */
    // Note: Standard ID is 11 bits (0x7FF)
    tx_header_local.Identifier = id & 0x7FFU;
    tx_header_local.DataLength = bytesToDlc(length);

    /* Add message to TX FIFO */
    if (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &tx_header_local, (uint8_t*)data) != HAL_OK) {
        return false;
    }

    return true;
}

/* ============================================================================
 * Reception
 * ============================================================================ */

bool FDCANCommunication::receive(uint32_t* id, uint8_t* data, uint32_t* length)
{
    if (id == nullptr || data == nullptr || length == nullptr || hfdcan == nullptr) {
        return false;
    }

    FDCAN_RxHeaderTypeDef rx_header = {0};

    /* Get message from RX FIFO0 - pass user buffer directly to avoid extra copy */
    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx_header, data) != HAL_OK) {
        return false; // FIFO empty or error
    }

    /* Extract and decode message */
    *id = rx_header.Identifier;
    // Convert DLC code to byte count
    *length = dlcToBytes((uint8_t)rx_header.DataLength);

    if (*length > FDCAN_MAX_PAYLOAD_SIZE) {
        *length = FDCAN_MAX_PAYLOAD_SIZE;
    }

    return true;
}

/* ============================================================================
 * Error Monitoring and Recovery
 * ============================================================================ */

uint32_t FDCANCommunication::getErrorStatus(void)
{
    if (hfdcan == nullptr || hfdcan->Instance == nullptr) return 0U;
    uint32_t psr = hfdcan->Instance->PSR;
    return decodeErrorStatus(psr);
}

bool FDCANCommunication::isBusOff(void) const
{
    if (hfdcan == nullptr || hfdcan->Instance == nullptr) return false;
    // Check internal state OR PSR register
    return is_bus_off || ((hfdcan->Instance->PSR & FDCAN_PSR_BO) != 0U);
}

bool FDCANCommunication::recoverFromBusOff(void)
{
    if (!isBusOff()) {
        return true;  /* Not in bus-off */
    }

    /* 1. Stop FDCAN */
    if (!deinit()) {
        return false;
    }

    /* 2. Wait for bus recovery (required ~1.3ms for a 500kbit/s bus, using 200ms is safe) */
    HAL_Delay(200);

    /* 3. Restart FDCAN using stored config and callbacks */
    if (!init(active_config, rx_callback, error_callback)) {
        return false;
    }

    is_bus_off = false;
    return true;
}

/* ============================================================================
 * Helper Functions (Static)
 * ============================================================================ */

uint32_t FDCANCommunication::bytesToDlc(uint8_t size)
{
    /* Convert byte count to DLC value (Optimized for standard lengths) */
    if (size <= 8U) return size;
    if (size <= 12U) return FDCAN_DLC_BYTES_12;
    if (size <= 16U) return FDCAN_DLC_BYTES_16;
    if (size <= 20U) return FDCAN_DLC_BYTES_20;
    if (size <= 24U) return FDCAN_DLC_BYTES_24;
    if (size <= 32U) return FDCAN_DLC_BYTES_32;
    if (size <= 48U) return FDCAN_DLC_BYTES_48;
    return FDCAN_DLC_BYTES_64;
}

uint32_t FDCANCommunication::dlcToBytes(uint8_t dlc)
{
    /* Lookup table for DLC to bytes conversion (CAN-FD spec) */
    static const uint32_t DLC_BYTES_TABLE[16U] = {
        0U,  1U,  2U,  3U,   4U,   5U,   6U,   7U,
        8U,  12U, 16U, 20U,  24U,  32U,  48U,  64U
    };

    if (dlc >= 16U) {
        return 0U;
    }

    return DLC_BYTES_TABLE[dlc];
}

uint32_t FDCANCommunication::decodeErrorStatus(uint32_t psr)
{
    uint32_t error_flags = 0U;

    /* Check Error Warning (EW) flag */
    if ((psr & FDCAN_PSR_EW) != 0U) {
        error_flags |= FDCAN_ERR_WARNING;
    }

    /* Check Error Passive (EP) flag */
    if ((psr & FDCAN_PSR_EP) != 0U) {
        error_flags |= FDCAN_ERR_PASSIVE;
    }

    /* Check Bus-Off (BO) flag */
    if ((psr & FDCAN_PSR_BO) != 0U) {
        error_flags |= FDCAN_BUS_OFF;
    }

    // The HAL error callback is typically triggered by IR flags, not just PSR
    // We rely on the HAL_FDCAN_ErrorStatusCallback to check IR if needed
    // The user can get more detail by reading hfdcan->Instance->IR inside the callback

    return error_flags;
}

/* ============================================================================
 * Interrupt Processing
 * ============================================================================ */

void FDCANCommunication::processRxFifo(void)
{
    if (rx_callback == nullptr || hfdcan == nullptr || hfdcan->Instance == nullptr) {
        return;
    }

    uint32_t id = 0U;
    uint8_t data[FDCAN_MAX_PAYLOAD_SIZE] = {0};
    uint32_t length = 0U;

    /* Process all available messages in RX FIFO0 */
    while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) != 0U) {

        if (!receive(&id, data, &length)) {
            // Error occurred (e.g., RX message lost)
            // Call error processor, which will then call the error_callback
            processError();
            return;
        }

        /* Invoke user callback with received data */
        rx_callback(id, data, length);
    }
}

void FDCANCommunication::processError(void)
{
    if (error_callback == nullptr || hfdcan == nullptr) return;

    // Read Protocol Status Register (PSR) for general state
    uint32_t psr_flags = getErrorStatus();

    // Read Interrupt Register (IR) for specific event
    uint32_t ir_flags = hfdcan->Instance->IR;

    uint32_t error_flags = psr_flags;

    if ((ir_flags & FDCAN_IT_BUS_OFF) != 0U) {
        error_flags |= FDCAN_BUS_OFF;
        is_bus_off = true;
    }
    if ((ir_flags & FDCAN_IT_RX_FIFO0_MESSAGE_LOST) != 0U) {
        error_flags |= FDCAN_ERR_FIFO0_OVERRUN;
    }
    if ((ir_flags & FDCAN_IT_ARB_PROTOCOL_ERROR) != 0U) {
        error_flags |= FDCAN_ERR_PROTOCOL_ARB;
    }
    if ((ir_flags & FDCAN_IT_DATA_PROTOCOL_ERROR) != 0U) {
        error_flags |= FDCAN_ERR_PROTOCOL_DATA;
    }

    /* Invoke error callback */
    error_callback(error_flags);

    /* HAL driver handles clearing interrupts in the IRQ handler context;
     * no explicit clearing needed here as processError() is called from the callback. */
}

/* ============================================================================
 * HAL Callback Implementations (C linkage)
 * ============================================================================ */

/**
 * @brief RX FIFO0 callback - called by HAL_FDCAN_IRQHandler
 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != 0U) {
        g_fdcan_comm.processRxFifo();
    }
}

/**
 * @brief Error status callback - called by HAL_FDCAN_IRQHandler
 */
void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan)
{
    // Check if the current handle matches the global instance's handle before processing
    // This is required only if supporting multiple FDCAN instances.
    // For single instance, we assume it's g_fdcan_comm.
    g_fdcan_comm.processError();
}
