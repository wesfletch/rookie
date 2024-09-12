#ifndef SERIAL_PORT_HPP
#define SERIAL_PORT_HPP

#include <optional>
#include <string>

namespace mobility_driver 
{

class SerialPort
{
public:

    SerialPort() = default;

    ~SerialPort();

    bool configure(int frequency);

    std::optional<std::string> spinOnce();

    void spin();

    bool write(std::string out);

    bool isConfigured() { return configured; }

protected:
private:

    /**
     * NOTE: `serial_port` not usable unless isConfigured();
     */
    int serial_port;

    /**
     * NOTE: `frequency` not usable unless isConfigured();
     */
    int frequency;

    bool configured = false;

}; // class SerialPort

} // namespace mobility_driver


#endif // SERIAL_PORT_HPP
