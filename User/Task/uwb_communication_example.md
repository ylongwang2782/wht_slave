# UWB Communication Task Usage Example

The UWB communication task provides a message queue-based interface for sending and receiving UWB data. This allows other tasks to communicate via UWB without directly handling DW1000 operations.

## Key Features

1. **Message Queue Based**: Uses FreeRTOS message queues for thread-safe communication
2. **Non-blocking Operations**: Tasks won't block when sending data
3. **Callback Support**: Optional callback function for immediate data processing
4. **Queue Management**: Functions to check queue status and clear queues
5. **Automatic UDP Forwarding**: Received UWB data is automatically forwarded via UDP

## API Functions

### Initialization
```c
#include "uwb_task.h"

// Initialize the UWB communication task (call once during startup)
UWB_Task_Init();
```

### Sending Data
```c
// Send UWB data with optional delay
uint8_t data[] = {0xC5, 0x01, 'H', 'E', 'L', 'L', 'O'};
int result = UWB_SendData(data, sizeof(data), 0);  // No delay

// Send with 100ms delay
int result2 = UWB_SendData(data, sizeof(data), 100);

// Return values:
// 0  - Success
// -1 - Parameter error (NULL data, invalid length)
// -3 - Queue full or timeout
```

### Receiving Data
```c
// Method 1: Polling (non-blocking)
uwb_rx_msg_t msg;
if (UWB_ReceiveData(&msg, 0) == 0) {
    // Process received data
    // msg.data contains the actual UWB frame data
    // msg.data_len contains frame length
    // msg.timestamp contains reception timestamp
    // msg.status_reg contains DW1000 status register value
}

// Method 2: Blocking with timeout
if (UWB_ReceiveData(&msg, 1000) == 0) {  // Wait up to 1 second
    // Process received data
}
```

### Callback Method
```c
// Define callback function
void on_uwb_data_received(const uwb_rx_msg_t *msg) {
    // Process data immediately when received
    // This runs in the UWB task context, keep processing minimal
    elog_i("APP", "UWB data received: %d bytes", msg->data_len);
}

// Register callback
UWB_SetRxCallback(on_uwb_data_received);
```

### Queue Management
```c
// Check queue status
int tx_count = UWB_GetTxQueueCount();  // Messages waiting to be sent
int rx_count = UWB_GetRxQueueCount();  // Messages waiting to be read

// Clear queues if needed
UWB_ClearTxQueue();  // Clear pending send messages
UWB_ClearRxQueue();  // Clear pending receive messages
```

### Configuration
```c
// Reconfigure UWB parameters (useful for changing modes)
int result = UWB_Reconfigure();
if (result == 0) {
    elog_i("APP", "UWB reconfigured successfully");
}
```

## Message Structures

```c
// Received message structure
typedef struct {
    uint16_t data_len;             // Length of received data
    uint8_t data[FRAME_LEN_MAX];   // Actual UWB frame data
    uint32_t timestamp;            // Reception timestamp (OS ticks)
    uint32_t status_reg;           // DW1000 status register value
} uwb_rx_msg_t;

// Callback function type
typedef void (*uwb_rx_callback_t)(const uwb_rx_msg_t *msg);
```

## Integration Examples

### Periodic Beacon Transmission
```c
void beacon_task(void *argument) {
    uint8_t beacon[] = {0xC5, 0x00, 'B', 'E', 'A', 'C', 'O', 'N', 0x00, 0x00};
    uint8_t sequence = 0;
    
    while (1) {
        beacon[1] = sequence++;  // Update sequence number
        UWB_SendData(beacon, sizeof(beacon), 0);
        osDelay(1000);  // Send every second
    }
}
```

### Data Collection Task
```c
void data_collector_task(void *argument) {
    uwb_rx_msg_t msg;
    
    while (1) {
        if (UWB_ReceiveData(&msg, 100) == 0) {
            // Log received data
            elog_i("COLLECTOR", "Received frame type: 0x%02X, len: %d", 
                   msg.data[0], msg.data_len);
            
            // Process based on frame type
            switch (msg.data[0]) {
                case 0xC5:  // Beacon frame
                    process_beacon(&msg);
                    break;
                case 0x41:  // Data frame
                    process_data(&msg);
                    break;
                default:
                    elog_w("COLLECTOR", "Unknown frame type: 0x%02X", msg.data[0]);
                    break;
            }
        }
    }
}
```

### Response to UDP Commands
```c
void process_udp_command(const char *command, const uint8_t *data, uint16_t len) {
    if (strcmp(command, "UWB_SEND") == 0) {
        // Send received UDP data via UWB
        UWB_SendData(data, len, 0);
    } else if (strcmp(command, "UWB_BEACON") == 0) {
        // Send a beacon frame
        uint8_t beacon[] = {0xC5, 0x01, 'C', 'M', 'D', 0x00, 0x00};
        UWB_SendData(beacon, sizeof(beacon), 0);
    }
}
```

## Configuration

- **FRAME_LEN_MAX**: 127 bytes (maximum UWB frame length)
- **TX_QUEUE_SIZE**: 10 messages (sending queue)
- **RX_QUEUE_SIZE**: 10 messages (receiving queue)
- **UDP Forward**: Automatic forwarding to 192.168.0.103:9000

## UWB Frame Format

The task works with standard 802.15.4 frames:
```
Byte 0:     Frame type (0xC5 for beacon, 0x41 for data, etc.)
Byte 1:     Sequence number
Byte 2-N:   Payload data
Byte N+1,N+2: CRC (automatically handled by DW1000)
```

## Thread Safety

- All API functions are thread-safe
- Multiple tasks can send data simultaneously
- Only one callback function can be registered
- The callback runs in the UWB task context

## Error Handling

- Functions return error codes for parameter validation
- Queue full conditions are handled gracefully
- DW1000 errors are logged and handled internally
- Automatic recovery from transmission/reception errors

## Performance Considerations

- UWB task runs with normal priority
- Minimal delay (1ms) in main loop for efficiency
- Non-blocking operations prevent task starvation
- Automatic restart of reception after transmission 