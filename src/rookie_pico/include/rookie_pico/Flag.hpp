#ifndef FLAG_HPP
#define FLAG_HPP

#include <pico/sync.h>

class System;

/**
 * @brief A singleton "flag" that can be read by subsystems.
 * 
 * The idea here is that the "System" state machine needs to be able to 
 * signal to the subsystems it controls. So, it can set this flag to OK or NOT OK, 
 * and the subsystems  that share this flag (e.g., the MotorController) can check 
 * its status periodically. 
 */
class Flag
{
public:

    enum class STATE : uint8_t {
        OK = 0,
        STOP = 1
    };

    Flag()
    {
        mutex_init(&this->mtx);
    }

    /**
     * @brief bool() operator, when Flag is OK it's truthy
     * 
     * @return true Flag::STATE::OK
     * @return false not Flag::STATE::OK
     */
    explicit operator bool() const
    {
        return (this->state == Flag::STATE::OK);
    }

    Flag::STATE getState() 
    {
        Flag::STATE returned;
        mutex_enter_blocking(&this->mtx);
        returned = this->state;
        mutex_exit(&this->mtx);

        return returned;
    }


protected:
private:

    /**
     * Make System a friend of this class so that ONLY System
     * can change the STATE of the Flag.
     */
    friend System;
    void _setState(Flag::STATE new_state)
    {
        mutex_enter_blocking(&this->mtx);
        this->state = new_state;
        mutex_exit(&this->mtx);
    }

    /* Read/write protection for this->state. */
    mutex_t mtx;

    /* The current state of this Flag. */
    Flag::STATE state = Flag::STATE::STOP;

}; // class Flag


#endif // FLAG_HPP