#include "rclcpp/rclcpp.hpp"
//#include "motor_test/xl330_driver.hpp"
#include "motor_test/epos4_driver.hpp"
#include <thread>

using namespace std::chrono_literals;

enum class TestState {
    INIT,
    ACTIVATE,
    VELOCITY_TEST,
    POSITION_TEST,
    SHUTDOWN,
    ERROR,
    DONE
};

bool runEPOS4Test(EPOSDriver& driver)
{
    TestState state = TestState::INIT;
    bool ok = false;

    while (state != TestState::DONE)
    {
        switch (state)
        {
        // --------------------- INIT ---------------------
        case TestState::INIT:
            ok = driver.init();
            if (!ok) { state = TestState::DONE;  break; }
            state = TestState::ACTIVATE;
            break;

        // ------------------ ACTIVATE --------------------
        case TestState::ACTIVATE:
            ok = driver.activate();
            if (!ok) { state = TestState::ERROR; break; }
            state = TestState::VELOCITY_TEST;
            break;

        // ---------------- VELOCITY TEST -----------------
        case TestState::VELOCITY_TEST:
            ok = driver.setControlMode(VELOCITY_MODE);
            if (!ok) { state = TestState::ERROR; break; }

            ok = driver.setVelocity(10);
            if (!ok) { state = TestState::ERROR; break; }

            std::this_thread::sleep_for(3s);
            std::cout << "Velocity is " << driver.getVelocity() << std::endl;

            driver.stop();   // stop() сама проверяет state
            state = TestState::POSITION_TEST;
            break;

        // ---------------- POSITION TEST -----------------
        case TestState::POSITION_TEST:
            ok = driver.setControlMode(POSITION_MODE);
            if (!ok) { state = TestState::ERROR; break; }

            std::this_thread::sleep_for(3s);
            std::cout << "Position is " << driver.getPosition() << std::endl;

            driver.stop();
            state = TestState::SHUTDOWN;
            break;

        // ------------------ SHUTDOWN ---------------------
        case TestState::SHUTDOWN:
            driver.shutdown();  // shutdown() сама защитится от null
            state = TestState::DONE;
            break;

        // ---------------------- ERROR --------------------
        case TestState::ERROR:
            std::cerr << "!!! ERROR: stopping test early !!!" << std::endl;
            driver.shutdown();  // безопасно
            state = TestState::DONE;
            break;

        default:
            state = TestState::DONE;
        }
    }

    return true;
}


int main()
{   

    EPOSDriver EPOS4_driver;     //   Provide default constructor, or
    //   or Pass parameters manually.
    //  EPOSDriver d("EPOS4", "MAXON SERIAL V2", "USB", "USB0", 1, 1000000);

    runEPOS4Test(EPOS4_driver);
    
    return 0;
}