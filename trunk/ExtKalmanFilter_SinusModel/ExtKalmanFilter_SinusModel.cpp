/****************************************************************************************************************/
/*                                                                                                              */
/*   Morphomotion: A project researching on how morphology influences locomotion in robots.                     */
/*                                                                                                              */
/*   E X T E N D E D   K A L M A N   F I L T E R   S I N U S O I D A L   M O D E L   C L A S S                  */
/*                                                                                                              */
/*   Avinash Ranganath                                                                                          */
/*   Robotics Lab, Department of Systems Engineering and Automation                                             */
/*   University Carlos III de Mardid(UC3M)                                                                      */
/*   Madrid, Spain                                                                                              */
/*   E-mail: avinash.ranganath@uc3m.es                                                                          */
/*   http://roboticslab.uc3m.es/roboticslab/persona.php?id_pers=104                                             */
/*                                                                                                              */
/****************************************************************************************************************/


#include"ExtKalmanFilter_SinusModel.h"

ExtKalmanFilter_SinusModel::ExtKalmanFilter_SinusModel(const double delta_time)
{
    r = 0.1;
    //qf = 0.00000001;
    qf = 0.0001;

    dt = delta_time;
    omega = 0;

    K.set_size(2,1);

    X.set_size(2,1);
    //X << 0 << endr  //--Initial estimate
    //  << 0 << endr; //--Amplitude.

    H << 1 << 0 << endr;

    F.set_size(2,2);

    Q.set_size(2,2);

    P.set_size(2,2);
    //P << 1000 << 0 << endr
    //  << 0 << 1000 << endr;

    //--Identity Matrix
    I << 1 << 0 << endr
      << 0 << 1 << endr;

    reset_parameters();
}


void ExtKalmanFilter_SinusModel::reset_parameters()
{
    X(0,0) = 0;  //--Initial estimate
    X(1,0) = 0; //--Amplitude.

    P(0,0) = 1000;
    P(0,1) = 0;
    P(1,0) = 0;
    P(1,1) = 1000;
}


void ExtKalmanFilter_SinusModel::set_dt(const double delta_time)
{
    if(delta_time <= 0)
    {
        std::cerr << "Morphomotion Error: ExtKalmanFilter_SinusModel class." << std::endl
              << "void set_dt(const double) method." << std::endl
              << "Invalide delta_time: " << delta_time << std::endl;
        exit(1);
    }
    else
    {
        dt = delta_time;
    }
}


void ExtKalmanFilter_SinusModel::set_omega(const double frequency)
{
    if(frequency <= 0)
    {
        std::cerr << "Morphomotion Error: ExtKalmanFilter_SinusModel class." << std::endl
              << "void set_omega(const double) method." << std::endl
              << "Invalide frequency: " << frequency << std::endl;
        exit(1);
    }
    else
    {
        omega = 2*M_PI*frequency;
    }
}


void ExtKalmanFilter_SinusModel::set_r(const double new_r)
{
    r = new_r;
}


void ExtKalmanFilter_SinusModel::set_qf(const double new_qf)
{
    qf = new_qf;
}

double ExtKalmanFilter_SinusModel::predict_and_update(const double y, const double t)
{
    double gama = omega*cos(omega*t);
    double Beta = 2*t + dt;

    F(0,0) = 1.0;
    F(0,1) = omega*cos(omega*t);
    F(1,0) = 0;
    F(1,1) = 1.0;

    Q(0,0) = (qf*gama*gama*dt*((3*t*t) + (3*t*dt) + (dt*dt)))/3.0;
    Q(0,1) = (qf*gama*dt*Beta)/2.0;
    Q(1,0) = (qf*gama*dt*Beta)/2.0;
    Q(1,1) = qf*dt;

    //--Predict
    X = F * X;
    P = F*P*trans(F) + Q;

    //--Calculate Kalman gain (Weight).
    //K = (P*trans(H)) / ((H*P)*trans(H) + r);
    K = P*trans(H);
    mat T = (H*P)*trans(H) + r;
    double d = T(0,0);
    K = K / d;

    //--Calculate the new estimate.
    X = X + K*(y - H*X);

    //--Update the State variance.
    P = (I - K*H)*P;

    return X(0,0);
}

