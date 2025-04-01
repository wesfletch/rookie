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

    void configure_tty(struct termios* tty);

    std::optional<std::string> spinOnce();

    void spin();

    void spin_jthread(std::stop_token stop_token);

    void push(std::string_view message);
    
    std::optional<std::string> pop();
    
    void start_jthread();
    
    void stop_jthread();
    
protected:
private:
    
    bool _write(std::string out);

    /**
     * NOTE: `serial_port` not usable unless isConfigured();
     */
    int serial_port;

    std::string device_name;

    std::mutex mtx_outbound;
    std::queue<std::string> outbound;

    std::mutex mtx_inbound;
    std::queue<std::string> inbound;

    std::jthread thread;

    /**
     * NOTE: `frequency` not usable unless isConfigured();
     */
    int frequency;

}; // class SerialPort

} // namespace mobility_driver


#endif // SERIAL_PORT_HPP
