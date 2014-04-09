//#include<math.h>
#include<fstream>
#include <vector>
#include <stdlib.h>

#include"ExtKalmanFilter_SinusModel.h"

#define SAMPLE_SIZE 7

std::vector<double> actual_error;


double calculate_random_uniform(double minimum, double maximum)
{
  double random = (double)rand()/(RAND_MAX+1.0);

  double random_uniform = minimum + (maximum-minimum)*random;

  return(random_uniform);
}


double calculate_random_normal(double mean, double standard_deviation)
{
  double random_normal = 0.0;

  const double pi = 4.0*atan(1.0);

  double random_uniform_1;
  double random_uniform_2;

  do
  {
    random_uniform_1 = (double)rand()/(RAND_MAX+1.0);

  }while(random_uniform_1 == 0.0);

  random_uniform_2 = (double)rand()/(RAND_MAX+1.0);

  // Box-Muller transformation

  random_normal = mean + sqrt(-2.0*log(random_uniform_1))*sin(2.0*pi*random_uniform_2)*standard_deviation;

  return(random_normal);
}

//AVERAGE
double Average(std::vector<double> v)
{
    double sum=0;
    for(unsigned int i=0;i<v.size();i++)
    {
        sum+=v[i];
    }
    return sum/v.size();
}

//DEVIATION
double Deviation(std::vector<double> v, double ave)
{
    double E=0;
    for(unsigned int i=0;i<v.size();i++)
    {
        E+=(v[i] - ave)*(v[i] - ave);
    }

    return sqrt(E/(double)v.size());
}


double mean_squared_error(std::vector<double> v)
{
    double E=0;
    for(unsigned int i=0;i<v.size();i++)
    {
        E+=v[i];
    }

    return sqrt(E/(2.0*(double)v.size()));
}


void read_from_file(std::vector<double>& time,
                    std::vector<double>& reference,
                    std::vector<double>& measured,
                    std::vector<double>& filtered)
{
    std::ifstream iFile;
    iFile.open("../Evaluation_Files/data.dat", std::fstream::in);

    double number;

    while(!iFile.eof())
    {
        iFile >> number;
        time.push_back(number);

        iFile >> number;
        reference.push_back(number);

        iFile >> number;
        measured.push_back(number);

        iFile >> number;
        filtered.push_back(number);
    }
    time.erase(time.end()-1);
    reference.erase(reference.end()-1);
    measured.erase(measured.end()-1);
    filtered.erase(filtered.end()-1);

    iFile.close();
}


double Re_Evaluate()
{
    std::fstream oFile;
    double actual = 10;
    std::vector<double> reference;
    std::vector<double> measured;
    std::vector<double> filtered;
    double estimate_measured;
    double average_estimate;
    std::vector<double> error_measured;
    std::vector<double> error_average;

    double amplitude = 60;
    double frequency = 0.9999;
    double phase = 0;
    double offset = 0;
    std::vector<double> time;
    double resolution = 0.01;

    ExtKalmanFilter_SinusModel K_Sinus(resolution);
    K_Sinus.set_omega(frequency);
    K_Sinus.set_r(0.1);
    K_Sinus.set_qf(0.0001);

    remove("../Evaluation_Files/output.dat");
    oFile.open("../Evaluation_Files/output.dat", std::fstream::in | std::fstream::out | std::fstream::app);

    read_from_file(time, reference, measured, filtered);

    oFile << "Time" << "  Actual" << "  Reference" << "  Measured" << "  Estimate" << "  Filtered" << std::endl;

    for(unsigned long i=0; i<time.size(); i++)
    {
        actual = amplitude * sin(2*M_PI*frequency*time[i] + ((phase * M_PI)/180.0)) + offset;
        //measured = actual + calculate_random_uniform(-30.0, 30.0);
        //actual_error.push_back((actual-measured)*(actual-measured));

        estimate_measured = K_Sinus.predict_and_update(measured[i], time[i]);

        std::vector<double> sub_vector;
        for(unsigned int j=i, k=0; j>=0 && k<= SAMPLE_SIZE; j++,k++ )
        {
            sub_vector.push_back(measured[j]);
        }
        average_estimate = Average(sub_vector);

        error_measured.push_back((measured[i]-estimate_measured)*(measured[i]-estimate_measured));
        error_average.push_back((measured[i]-average_estimate)*(measured[i]-average_estimate));

        oFile << time[i] << " " << actual << " " << reference[i] << " " <<
                 measured[i] << " " << estimate_measured << " " << filtered[i] <<
                 " " <<  average_estimate << std::endl;

        //std::cout << time[i] << " " << reference[i] << " " << measured[i] << " " << filtered[i] << std::endl;
    }

    oFile.close();

    std::cout << "Error Measured = " << mean_squared_error(error_measured)
              << "    Error Average: " << mean_squared_error(error_average) << std::endl;

    return mean_squared_error(error_measured);
}


double epoch()
{
    std::fstream oFile;
    double actual = 10;
    double measured;
    double estimate;
    std::vector<double> error;

    double amplitude = 80;
    double frequency = 1.0;
    double phase = 0;
    double offset = 0;
    double time = 0;
    unsigned long steps = 100;
    unsigned int cycles = 50;
    double resolution = 1.0/(double)steps;
    //resolution = resolution * 4.0;  //--TODO: To be removed.
    double new_time;
    double delta_time = resolution;

    ExtKalmanFilter_SinusModel K_Sinus(resolution);
    K_Sinus.set_omega(frequency);


    remove("../Evaluation_Files/output.dat");
    oFile.open("../Evaluation_Files/output.dat", std::fstream::in | std::fstream::out | std::fstream::app);

    for(unsigned long i=0; i<steps*cycles; i++)
    {
        actual = amplitude * sin(2*M_PI*frequency*time + ((phase * M_PI)/180.0)) + offset;
        measured = actual + calculate_random_uniform(-30.0, 30.0);
        actual_error.push_back((actual-measured)*(actual-measured));

        estimate = K_Sinus.predict_and_update(measured, time);
        error.push_back((actual-estimate)*(actual-estimate));

        oFile << time << " " << actual << " " << measured << " " << estimate << std::endl;

        time = time + resolution;

        /*
        //new_time = time + resolution + calculate_random_uniform(0, 3.0*resolution); //--INCORRECT.
        //new_time = time + resolution*(((int)calculate_random_uniform(0, 6.0)%6)+1);
        //new_time = time + resolution*(i%6 +1);

        double factor = calculate_random_uniform(1.0, 1.2);
        //double factor = calculate_random_normal(1.0, 0.15);
        //double factor = 1.0;

        //unsigned int factor = (((int)calculate_random_uniform(0, 6.0)%6)+1);
        //unsigned int factor = i%6 +1;

        //std::cout << "Factor: " << factor << std::endl;
        new_time = time + (resolution * factor);
        //new_time = time + (resolution * 1.0);

        //delta_time = new_time - time;
        //K_Sinus.set_dt(resolution*(i%6 +1));
        //for(unsigned int j=0; j<10; j++)
        //{
        //    estimate_sinus = K_Sinus.predict_and_update(measured, new_time);
        //}
        time = new_time;
        */
    }

    //std::cout << "Error_Static: " << error_static/(2.0*steps*cycles) << std::endl;
    //std::cout << "Error_Linear: " << error_linear/(2.0*steps*cycles) << std::endl;
    //std::cout << "Error_Sinus : " << error_sinus/(2.0*steps*cycles) << std::endl;

    oFile.close();

    return mean_squared_error(error);
}


int main()
{
    //--Initialize random seed
    srand (time(NULL));

    unsigned int trials = 1;
    std::vector<double> errors;

    for(unsigned i=0; i<trials; i++)
    {
        //errors.push_back(epoch());
        errors.push_back(Re_Evaluate());
    }

    //std::cout << "Mean Error = " << Average(errors) << "    Standard Deviation: " << Deviation(errors, Average(errors)) << std::endl;
    //std::cout << "Actual Error = " << mean_squared_error(actual_error) << std::endl;

    return 0;
}
