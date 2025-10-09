/******************************************************************************
 *  File Name:
 *    sim_tcp_server.hpp
 *
 *  Description:
 *    Object-oriented TCP/IP server for simulator builds. Supports multiple
 *    ports and arbitrary binary data transmission.
 *
 *  2025 | Brandon Braun | brandonbraun653@gmail.com
 *****************************************************************************/

#pragma once
#ifndef ORBIT_ESC_SIM_TCP_SERVER_HPP
#define ORBIT_ESC_SIM_TCP_SERVER_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <vector>

namespace Orbit::Sim::TCP
{
  /*---------------------------------------------------------------------------
  Forward Declarations
  ---------------------------------------------------------------------------*/
  class Server;

  /*---------------------------------------------------------------------------
  Types
  ---------------------------------------------------------------------------*/

  /**
   * @brief Callback function type for received data
   * @param server Reference to the server instance that received the data
   * @param data Pointer to received data
   * @param size Number of bytes received
   */
  using DataReceivedCallback = std::function<void( Server &server, const void *data, size_t size )>;

  /**
   * @brief Configuration structure for TCP server instances
   */
  struct ServerConfig
  {
    uint16_t             port;              ///< TCP port to listen on
    size_t               rx_buffer_size;    ///< Receive buffer size in bytes
    size_t               tx_buffer_size;    ///< Transmit buffer size in bytes
    DataReceivedCallback rx_callback;       ///< Callback for received data
    bool                 auto_reconnect;    ///< Automatically reconnect on disconnect
    size_t               socket_backlog;    ///< Socket listen backlog

    ServerConfig() :
        port( 55001 ), rx_buffer_size( 1024 ), tx_buffer_size( 1024 ), rx_callback( nullptr ), auto_reconnect( true ),
        socket_backlog( 1 )
    {
    }
  };


  /*---------------------------------------------------------------------------
  TCP Server Class
  ---------------------------------------------------------------------------*/

  /**
   * @brief Object-oriented TCP server for simulator communication
   *
   * This class provides a flexible TCP server implementation that can handle
   * multiple ports simultaneously and transmit arbitrary binary data. Each
   * server instance runs in its own thread and manages a single client
   * connection per port.
   */
  class Server
  {
  public:
    /**
     * @brief Constructor
     * @param config Server configuration
     */
    explicit Server( const ServerConfig &config );

    /**
     * @brief Destructor - automatically stops the server
     */
    ~Server();

    /**
     * @brief Starts the TCP server thread
     * @return true Server started successfully
     * @return false Failed to start server
     */
    bool start();

    /**
     * @brief Stops the TCP server thread
     */
    void stop();

    /**
     * @brief Sends binary data to the connected client
     * @param data Pointer to data to send
     * @param size Number of bytes to send
     * @return true Data queued for transmission
     * @return false Failed to queue data (server not running or buffer full)
     */
    bool sendData( const void *data, size_t size );

    /**
     * @brief Checks if the server is running
     * @return true Server thread is active
     * @return false Server thread is not running
     */
    bool isRunning() const;

    /**
     * @brief Checks if a client is connected
     * @return true Client is connected
     * @return false No client connected
     */
    bool isClientConnected() const;

    /**
     * @brief Gets the configured port number
     * @return Port number
     */
    uint16_t getPort() const;

    /**
     * @brief Updates the receive callback
     * @param callback New callback function
     */
    void setRxCallback( DataReceivedCallback callback );

  private:
    struct SocketContext;
    struct Impl;

    std::unique_ptr<Impl> m_impl;
  };

  /*---------------------------------------------------------------------------
  Server Manager
  ---------------------------------------------------------------------------*/

  /**
   * @brief Manages multiple TCP server instances
   */
  class ServerManager
  {
  public:
    /**
     * @brief Gets the singleton instance
     * @return Reference to the server manager
     */
    static ServerManager &getInstance();

    /**
     * @brief Creates a new server instance
     * @param config Server configuration
     * @return Pointer to created server, or nullptr on failure
     */
    std::shared_ptr<Server> createServer( const ServerConfig &config );

    /**
     * @brief Gets an existing server by port
     * @param port Port number
     * @return Pointer to server, or nullptr if not found
     */
    std::shared_ptr<Server> getServer( uint16_t port );

    /**
     * @brief Removes a server instance
     * @param port Port number of server to remove
     * @return true Server removed successfully
     * @return false Server not found
     */
    bool removeServer( uint16_t port );

    /**
     * @brief Gets all active server ports
     * @return Vector of active port numbers
     */
    std::vector<uint16_t> getActivePorts() const;

  private:
    ServerManager()                                   = default;
    ~ServerManager()                                  = default;
    ServerManager( const ServerManager & )            = delete;
    ServerManager &operator=( const ServerManager & ) = delete;

    mutable std::mutex                   m_mutex;
    std::vector<std::shared_ptr<Server>> m_servers;
  };


}    // namespace Orbit::Sim::TCP

#endif /* !ORBIT_ESC_SIM_TCP_SERVER_HPP */
