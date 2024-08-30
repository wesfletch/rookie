#ifndef FLAG_HPP
#define FLAG_HPP

#include  <pico/sync.h>

class System;

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

    explicit operator bool() const
    {
        if (this->state == Flag::STATE::OK) {
            return true;
        } else {
            return false;
        }
    }

    Flag::STATE getState() 
    {
        Flag::STATE returned;
        mutex_enter_blocking(&this->mtx);
        returned = this->state;
        mutex_exit(&this->mtx);

        return returned;
    }

    bool ok()
    {
        if (this->state == Flag::STATE::OK) {
            return true;
        } else {
            return false;
        }
    }

protected:
private:

    friend System;
    void _setState(Flag::STATE new_state)
    {
        mutex_enter_blocking(&this->mtx);
        this->state = new_state;
        mutex_exit(&this->mtx);
    }

    mutex_t mtx;

    Flag::STATE state = Flag::STATE::STOP;

}; // class Flag


#endif // FLAG_HPP