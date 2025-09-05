#ifndef SERIAL_PORT_HPP
#define SERIAL_PORT_HPP

#include <optional>
#include <queue>
#include <string>
#include <string_view>
#include <thread>

namespace mobility_driver 
{

class SerialPort
{
public:

    SerialPort(std::string_view device_name, const int& frequency);

    ~SerialPort();

    std::optional<std::string> spinOnce();
    
    void spin();
    
    /**
    * \brief Enqueue a message to be sent to the serial device.
    * 
    * \param message Message to be sent
    */
    void push(std::string_view message);
    
    /**
    * \brief Get a message FROM the serial device; FIFO.
    * 
    * \return std::optional<std::string>
    */
    std::optional<std::string> pop();
    
    void start_jthread();
    
    void stop_jthread();
    
    void spin_jthread(std::stop_token stop_token);

protected:
private:
    
    bool _write(std::string out);

    void _configure_tty(struct termios* tty);

    // File descriptor of the serial device.
    int serial_port;

    std::string device_name;
    
    int frequency;

    std::mutex mtx_outbound;
    std::queue<std::string> outbound;

    std::mutex mtx_inbound;
    std::queue<std::string> inbound;

    std::jthread thread;


}; // class SerialPort

} // namespace mobility_driver


#endif // SERIAL_PORT_HPP
