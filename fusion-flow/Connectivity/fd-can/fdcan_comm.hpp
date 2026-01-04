/*
 * fdcan_comm.hpp
 *
 * Improved FDCAN communication library for STM32G4
 */

#ifndef FD_CAN_FDCAN_COMM_HPP_
#define FD_CAN_FDCAN_COMM_HPP_

#include <cstdint>
#include <cstring>
#include "fdcan.h"
#include "stm32g0xx_hal_def.h"
#include "stm32g0xx_hal_fdcan.h"

/* FDCAN Configuration Constants */
#define FDCAN_MAX_PAYLOAD_SIZE    64U

/* FDCAN Callback Types */
typedef void (*FDCANRxCallback)(uint32_t id, const uint8_t* data, uint32_t length);
typedef void (*FDCANErrorCallback)(uint32_t error_flags);

/* FDCAN Error Status Flags (decoded from PSR) */
#define FDCAN_ERR_WARNING       (1U << 0U)   /* Error warning state (TEC/REC > 96) */
#define FDCAN_ERR_PASSIVE       (1U << 1U)   /* Error passive state (TEC/REC > 127) */
#define FDCAN_BUS_OFF           (1U << 2U)   /* Bus-off state - requires restart (TEC > 255) */
#define FDCAN_ERR_BUS_ACTIVE    (1U << 3U)   /* Back to active state */
#define FDCAN_ERR_PROTOCOL_ARB  (1U << 4U)   /* Protocol error during arbitration */
#define FDCAN_ERR_PROTOCOL_DATA (1U << 5U)   /* Protocol error during data phase */
#define FDCAN_ERR_FIFO0_OVERRUN (1U << 6U)   /* RX FIFO 0 Overrun */

/**
 * @struct FDCANConfig
 * @brief Structure for consolidating all FDCAN setup parameters.
 */
struct FDCANConfig {
    // RX Filter Configuration (Standard ID only for simplicity)
    uint32_t filter_id1;
    uint32_t filter_id2;
    uint32_t filter_type; // FDCAN_FILTER_DUAL / MASK / RANGE

    // TX Header Configuration
    bool brs_on;
};

/**
 * @class FDCANCommunication
 * @brief FDCAN communication library for STM32G4 with bus-off recovery and callbacks.
 */
class FDCANCommunication {
public:
    /**
     * @brief Constructor - Requires the FDCAN handle pointer.
     */
    FDCANCommunication(FDCAN_HandleTypeDef* handle);

    /**
     * @brief Initialize FDCAN, set up filters, and start peripheral.
     * * @param config Configuration structure for filters and TX header defaults.
     * @param rx_cb RX callback (called when message received).
     * @param error_cb Error callback (called on error/status change).
     * @return true if successful, false on error.
     */
    bool init(const FDCANConfig& config, FDCANRxCallback rx_cb, FDCANErrorCallback error_cb);

    /**
     * @brief Deinitialize FDCAN peripheral.
     * @return true if successful.
     */
    bool deinit(void);

    /**
     * @brief Transmit FDCAN message (non-blocking). Thread-safe.
     * * @param id CAN message ID (11-bit standard: 0x000-0x7FF).
     * @param data Data buffer pointer.
     * @param length Data length (0-64 bytes, auto-capped).
     * @return true if transmission queued successfully.
     */
    bool transmit(uint32_t id, const uint8_t* data, uint32_t length);

    /**
     * @brief Receive FDCAN message (non-blocking from RX FIFO0).
     *
     * @param id Pointer to store received CAN ID.
     * @param data Pointer to buffer for data (must be FDCAN_MAX_PAYLOAD_SIZE minimum).
     * @param length Pointer to store data length (in bytes).
     * @return true if message received, false if FIFO empty.
     */
    bool receive(uint32_t* id, uint8_t* data, uint32_t* length);

    /**
     * @brief Get current error status (decoded flags).
     * @return Bitmask of FDCAN_ERR_* flags.
     */
    uint32_t getErrorStatus(void);

    /**
     * @brief Check if in bus-off state.
     * @return true if bus-off, false otherwise.
     */
    bool isBusOff(void) const;

    /**
     * @brief Recover from bus-off condition.
     * @return true if recovery successful.
     */
    bool recoverFromBusOff(void);

    // --- Internal ISR Processing (Called by HAL Callbacks) ---

    void processRxFifo(void);
    void processError(void);

    // --- Static Utility Functions ---
    static uint32_t bytesToDlc(uint8_t size);
    static uint32_t dlcToBytes(uint8_t dlc);
    static uint32_t decodeErrorStatus(uint32_t psr);

private:
    // Helper to configure filter (used internally by init/recover)
    bool configRxFilter(const FDCANConfig& config);

    // Helper to configure TX header (used internally by init/recover)
    void configTxHeader(const FDCANConfig& config);

    FDCAN_HandleTypeDef* const hfdcan; // Now const, initialized in constructor
    FDCANRxCallback rx_callback;
    FDCANErrorCallback error_callback;
    FDCAN_TxHeaderTypeDef tx_header_cache; // Cached TX header
    bool is_initialized;
    bool is_bus_off;
    FDCANConfig active_config; // Store configuration for restart/recovery
};

/* Global instance for interrupt handler access - requires manual linkage */
// The user must define this instance: e.g., FDCANCommunication g_fdcan_comm(&hfdcan2);
extern FDCANCommunication g_fdcan_comm;

#endif /* FD_CAN_FDCAN_COMM_HPP_ */
