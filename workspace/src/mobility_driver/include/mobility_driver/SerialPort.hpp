#ifndef SERIAL_PORT_HPP
#define SERIAL_PORT_HPP

#include <optional>
#include <string>

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

    bool write(std::string out);

protected:
private:

    /**
     * NOTE: `serial_port` not usable unless isConfigured();
     */
    int serial_port;

    std::string device_name;

    /**
     * NOTE: `frequency` not usable unless isConfigured();
     */
    int frequency;

}; // class SerialPort

} // namespace mobility_driver


#endif // SERIAL_PORT_HPP
