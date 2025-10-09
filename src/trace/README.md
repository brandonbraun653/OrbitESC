# OrbitESC Trace Module

A generalized trace module that allows shipping data from anywhere in the project with simple commands. Supports both embedded microcontroller and desktop simulator environments with different serialization formats.

## Features

- **Unified Interface**: Single API for both embedded and simulator environments
- **Enum-based Registration**: Simple trace type enumeration with callback registration
- **Rate Limiting**: Built-in sample rate control to manage data volume
- **Thread-safe**: Safe for use in multi-threaded environments
- **Memory Efficient**: Uses ETL containers for embedded compatibility
- **Dual Serialization**: 
  - Embedded: COBS + nanopb protocol buffers
  - Simulator: Binary packed structures for Matlab communication

## Architecture

### Core Components

1. **TraceTypes** (`trace_types.hpp`): Core type definitions and data structures
2. **Registry** (`trace_registry.hpp/cpp`): Callback registration and management
3. **Interface** (`trace_interface.hpp/cpp`): Unified API for sending trace data
4. **Serializers**: Environment-specific serialization
   - `trace_embedded.hpp/cpp`: COBS + nanopb for embedded systems
   - `trace_simulator.hpp/cpp`: Binary packed structures for simulator
5. **Examples** (`trace_examples.hpp/cpp`): Usage examples and integration patterns

### Data Flow

```
Application Code
       ↓
   sendTrace()
       ↓
   Interface::sendTrace()
       ↓
   Serializer::serialize()
       ↓
   Registry::executeCallbacks()
       ↓
   Registered Callbacks
       ↓
   Communication Layer (Serial/TCP)
```

## Usage

### Basic Setup

```cpp
#include <src/trace/trace_interface.hpp>

// Initialize the trace system
Orbit::Trace::initializeTraceSystem();

// Send trace data
MyDataStruct data = {1.0f, 2.0f, 3.0f};
Orbit::Trace::sendTrace(TraceType::MOTOR_CURRENT_MEASUREMENTS, &data, sizeof(data));
```

### Callback Registration

```cpp
// Register a callback for a specific trace type
bool myCallback(TraceType type, const void* data, size_t size, uint32_t timestamp_us)
{
    // Process the trace data
    return true;
}

// Register with rate limiting (1ms sample rate)
Orbit::Trace::registerTraceCallback(TraceType::MOTOR_CURRENT_MEASUREMENTS, 
                                     myCallback, 1000);
```

### Environment-Specific Callbacks

#### Embedded Systems
```cpp
// Use existing serial communication
bool embeddedCallback(TraceType type, const void* data, size_t size, uint32_t timestamp_us)
{
    auto* serial_server = Orbit::Serial::Config::getSerialServer();
    return serial_server->write(data, size, 1000) == static_cast<int>(size);
}
```

#### Simulator
```cpp
// Use TCP server for Matlab communication
bool simulatorCallback(TraceType type, const void* data, size_t size, uint32_t timestamp_us)
{
    auto& tcp_manager = Orbit::Sim::TCP::ServerManager::getInstance();
    auto tcp_server = tcp_manager.getServer(37219);
    return tcp_server->sendData(data, size);
}
```

## Trace Types

The module supports predefined trace types:

```cpp
enum class TraceType : uint8_t
{
    INVALID = 0,
    
    // Motor control traces
    MOTOR_CURRENT_MEASUREMENTS,
    MOTOR_VOLTAGE_COMMANDS,
    MOTOR_SPEED_ESTIMATE,
    MOTOR_POSITION_ESTIMATE,
    
    // Control system traces
    CONTROL_ERROR_SIGNALS,
    CONTROL_OUTPUT_SIGNALS,
    CONTROL_REFERENCE_SIGNALS,
    
    // System monitoring traces
    SYSTEM_TEMPERATURE,
    SYSTEM_VOLTAGE,
    SYSTEM_CURRENT,
    SYSTEM_STATUS,
    
    // Custom user traces
    USER_CUSTOM_1,
    USER_CUSTOM_2,
    USER_CUSTOM_3,
    
    COUNT
};
```

## Serialization Formats

### Embedded (COBS + nanopb)
- Uses existing protocol buffer definitions
- COBS encoding for reliable serial transmission
- Compatible with existing serial communication system

### Simulator (Binary Packed)
- Simple binary structure with magic bytes
- Checksum verification
- Optimized for Matlab communication

## Integration Patterns

### Control Loop Integration
```cpp
void controlLoop()
{
    // ... control calculations ...
    
    // Send trace data (rate limited by registered callbacks)
    MotorCurrentData current_data = {ia, ib, ic, idc};
    Orbit::Trace::sendTrace(TraceType::MOTOR_CURRENT_MEASUREMENTS, 
                           &current_data, sizeof(current_data));
}
```

### Conditional Tracing
```cpp
void conditionalTracing()
{
    if (system_healthy)
    {
        Orbit::Trace::setTraceEnabled(TraceType::MOTOR_CURRENT_MEASUREMENTS, true);
    }
    else
    {
        Orbit::Trace::setTraceEnabled(TraceType::MOTOR_CURRENT_MEASUREMENTS, false);
    }
}
```

### Rate-Limited Tracing
```cpp
// Register callback with 1ms sample rate
Orbit::Trace::registerTraceCallback(TraceType::MOTOR_CURRENT_MEASUREMENTS, 
                                   callback, 1000);
```

## Performance Considerations

- **Memory Usage**: Fixed-size ETL containers, no dynamic allocation
- **CPU Overhead**: Minimal serialization overhead, rate limiting reduces load
- **Thread Safety**: Simple spinlock implementation for embedded systems
- **Data Volume**: Rate limiting prevents overwhelming communication channels

## Configuration

### Build Configuration
- **SIMULATOR**: Enables simulator-specific serialization
- **ORBIT_TRACE_MODULE**: Enables trace module compilation

### Runtime Configuration
- Callback registration/unregistration
- Enable/disable specific trace types
- Sample rate adjustment per trace type

## Dependencies

### Embedded
- Aurora logging library
- Chimera threading library
- nanopb protocol buffer library
- COBS encoding library

### Simulator
- Aurora logging library
- Chimera threading library
- OrbitSimulator TCP server
- Standard C++ libraries

## Examples

See `trace_examples.cpp` for comprehensive usage examples including:
- System initialization
- Control loop integration
- Conditional tracing
- Rate-limited tracing
- Custom data structures

## Thread Safety

The trace module is thread-safe and can be used from multiple threads:
- Registry operations are protected by spinlocks
- Callback execution is atomic
- No shared mutable state between threads

## Error Handling

The module provides robust error handling:
- Input validation for all public functions
- Graceful degradation on serialization failures
- Logging of errors and warnings
- Callback execution continues even if individual callbacks fail
