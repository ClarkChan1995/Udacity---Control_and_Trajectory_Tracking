/**********************************************
 * Self-Driving Car Nano-degree - Udacity
 *  Created on: December 11, 2020
 *      Author: Mathilde Badoual
 **********************************************/

#include "pid_controller.h"
#include <vector>
#include <iostream>
#include <math.h>

using namespace std;

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kpi, double Kii, double Kdi, double output_lim_maxi, double output_lim_mini) {
   /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   **/
   this->Kpi = Kpi;
   this->Kii = Kii;
   this->Kdi = Kdi;
   this->output_lim_maxi = output_lim_maxi;
   this->output_lim_mini = output_lim_mini;
   cte0 = 0;
   I = 0;
}


void PID::UpdateError(double cte) {
   /**
   * TODO: Update PID errors based on cte.
   **/
   if(abs(delta_time) < 0.000001) return;
   double P = Kpi * cte;
   I += Kii * cte * delta_time;
   double D = Kdi * (cte - cte0) / delta_time;
   action = P + I + D;
   cte0 = cte;
}

double PID::TotalError() {
   /**
   * TODO: Calculate and return the total error
    * The code should return a value in the interval [output_lim_mini, output_lim_maxi]
   */
   double control = action;
   if (action < output_lim_mini) control = output_lim_mini;
   if (action > output_lim_maxi) control = output_lim_maxi;
   return control;
}

double PID::UpdateDeltaTime(double new_delta_time) {
   /**
   * TODO: Update the delta time with new value
   */
   delta_time = new_delta_time;
   return delta_time;
}