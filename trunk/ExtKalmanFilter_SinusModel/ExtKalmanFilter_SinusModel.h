/****************************************************************************************************************/
/*                                                                                                              */
/*   Morphomotion: A project researching on how morphology influences locomotion in robots.                     */
/*                                                                                                              */
/*   E X T E N D E D   K A L M A N   F I L T E R   S I N U S O I D A L   M O D E L   C L A S S   H E A D E R    */
/*                                                                                                              */
/*   Avinash Ranganath                                                                                          */
/*   Robotics Lab, Department of Systems Engineering and Automation                                             */
/*   University Carlos III de Mardid(UC3M)                                                                      */
/*   Madrid, Spain                                                                                              */
/*   E-mail: avinash.ranganath@uc3m.es                                                                          */
/*   http://roboticslab.uc3m.es/roboticslab/persona.php?id_pers=104                                             */
/*                                                                                                              */
/****************************************************************************************************************/
#ifndef EXTKALMANFILTER_SINUSMODEL_H
#define EXTKALMANFILTER_SINUSMODEL_H

#include<iostream>
#include<math.h>
#include "armadillo"

using namespace arma;

class ExtKalmanFilter_SinusModel
{
private:
    mat X; //--Estimated state
    mat H; //--Measurement matrix (i.e., mapping measurements onto state)
    mat F; //--State transition matrix (i.e., transition between states)
    mat Q; //--Process variance matrix (i.e., error due to process)
    mat P; //--State variance matrix (i.e., error of estimation)
    mat I; //--Identity matrix
    mat K; //--Kalman gain (Weight)

    double qf; //--Process variance (i.e., error due to process)
    double r; //--Measurement variance (i.e., error from measurements)

    double omega;
    double dt;

public:
    ExtKalmanFilter_SinusModel(const double);
    void reset_parameters();
    double predict_and_update(const double, const double);
    void set_dt(const double);
    void set_omega(const double);
    void set_r(const double);
    void set_qf(const double);
};

#endif
