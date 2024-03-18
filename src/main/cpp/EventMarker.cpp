// #include "EventMarker.h"
// #include <chrono>

// using namespace EM;

// EventMarkers::EventMarkers(RobotContainer rc)
// : m_rc(rc.Get()) {}

// void EventMarkers::EMIntake() 
// {

//     if(m_rc->shooter.GetBottomArmEncoderVal() > 0.0 || m_rc->shooter.GetBottomArmEncoderVal() < 0.0 && m_rc->shooter.GetTopArmEncoderVal() > 0.0 || m_rc->shooter.GetTopArmEncoderVal() < 0.0){

//         m_rc->shooter.MoveToAngleDeg(0.0);

//     }
//     else if(m_rc->shooter.GetBottomArmEncoderVal() == 0.0 && m_rc->shooter.GetTopArmEncoderVal() == 0.0 && sensor == false){
//         time_t now;
//         double m_time = time(&now);
//         double dtime = 0.0;
//         while(dtime < 1){
//             dtime = m_time - time(&now);
//             m_rc->shooter.RunIntakeMotorPercent(0.2);
//         }

//     }
        
// }

// void EventMarkers::EMShoot()
// {

//     while(sensor == true)
//     {
//         m_rc->shooter.RunShooterPercent(50.0);
//     }

// }