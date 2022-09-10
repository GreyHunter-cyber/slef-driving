#include <stdlib.h>
#include <iomanip>
#include <math.h>
#include <stdio.h>
#include <limits>
#include <iostream>

#include "PID.h"

using namespace std;

//////////////////////////////////////////////PID基类实现//////////////////////////////////////
PID::PID()
{	
	this->Kp = 0.0f;
	this->Ki = 0.0f;
	this->Kd = 0.0f;
	this->p_error = 0.0f;
	this->i_error = 0.0f;
	this->d_error = 0.0f;
	this->abs_saturation_value = MAX_OUTPUT;
	this->abs_windup = MAX_KI; //placehodler limit the I controller max value
	this->is_initialized = false;
}

PID::PID(float k_p, float k_i, float k_d, float max_output, float max_ki)
{
	this->Kp = k_p;
	this->Ki = k_i;
	this->Kd = k_d;
	this->p_error = 0.0f;
	this->i_error = 0.0f;
	this->d_error = 0.0f;

	this->abs_saturation_value = max_output;
	this->abs_windup = max_ki; //placehodler limit the I controller max value
	this->is_initialized = true;
	printf("PID init sucessful.\n");
}

void PID::Init(float k_p, float k_i, float k_d, float max_output, float max_ki)
{
	if (this->is_initialized == true)
	{
		return;
	}
	else
	{
		this->Kp = k_p;
		this->Ki = k_i;
		this->Kd = k_d;
		printf("[PID]:Load the parametre:(%3.3f , %3.3f ,%3.3f)\n",this->Kp,this->Ki,this->Kd);
		this->p_error = 0.;
		this->i_error = 0.;
		this->d_error = 0.;

		this->abs_saturation_value = max_output;
		this->abs_windup = max_ki; //placehodler limit the I controller max value
		this->is_initialized = true;

		this->change_kp = 0.1*k_p;
	    this->change_ki = 0.1*k_i;
	    this->change_kd = 0.1*k_d;
		printf("PID class init sucessful.\n");
	}
}

void PID::Init(const char* filename)
{
	if (this->is_initialized == true)
	{
		return;
	}
	else
	{
        // Load PID parametre from the .txt file
		FILE *pfPID = fopen(filename, "r");
		if (pfPID == NULL)
		{
			printf("ERROR!ERROR!	PID Parameter breakdown!!!\n");
			fclose(pfPID);
			return;
		}
		else
		{	
			int i = 0;
			float a[5] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
			while (!feof(pfPID))
			{
				fscanf(pfPID, "%f", &a[i]);
				i++;
			}
			this->Kp = a[0];
			this->Ki = a[1];
			this->Kd = a[2];									 //define PID coefficient value
			this->abs_saturation_value = a[3]; //define output limiter Value
			this->abs_windup = a[4]; //placehodler limit the I controller max value
		    printf("[PID]:Load the parametre:(%3.3f , %3.3f ,%3.3f)\n",this->Kp,this->Ki,this->Kd);
			this->change_kp = 0.1*Kp;
	        this->change_ki = 0.1*Ki;
	        this->change_kd = 0.1*Kd;
			printf("PID class init sucessful.\n");
		}
		fclose(pfPID);		
	}
	this->p_error = 0.0f;
	this->i_error = 0.0f;
	this->d_error = 0.0f;
	this->is_initialized = true;
}

///////////////////////////////////////////////PID_basic实现/////////////////////////////////////////////
PID_basic::PID_basic() : PID()
{
  prev_error = 0.0f;
}

PID_basic::PID_basic(float k_p, float k_i, float k_d, float max_output, float max_ki): PID(k_p, k_i, k_d, max_output, max_ki)
{
  prev_error = 0.0f;
}

bool PID_basic::UpdateError(float _error)
{
	if (this->is_initialized = false)
	{
		printf("PID is not init!\n");
		return false;
	}
	else
	{
		this->p_error = _error;
		this->i_error += _error;
		this->d_error = _error - prev_error;
		prev_error = _error;

		if (fabs(this->i_error) > this->abs_windup) // paying attention to integral windup
		{
			if (this->i_error > 0)
			{
				this->i_error = this->abs_windup;
			}
			else if (this->i_error < 0)
			{
				this->i_error = -(1.0) * this->abs_windup;
			}
		}
		return true;
	}
}

float PID_basic::TotalError()
{
	// int beta = 1;
	// if(this->p_error > 5.0 || this->p_error < -5.0)
	// {
	// 	beta = 0;
	// }
   float output = Kp * p_error + Ki * i_error + (Kd/(1+p_error*p_error)) * d_error;
   if(output > this->abs_saturation_value)
   {
       output = this->abs_saturation_value;
   }
   if(output < -1.0*(this->abs_saturation_value))
   {
       output = -1.0*(this->abs_saturation_value);
   }
   return output;
}

void PID_basic::Reset()
{
	prev_error = 0.0f;
}

///////////////////////////////////////////////PID_increment实现/////////////////////////////////////////////
PID_increment::PID_increment() : PID()
{
  prev_error[0] = 0.0f;
  prev_error[1] = 0.0f;
  prev_error[2] = 0.0f;
  u_k = 0.0f;
}

PID_increment::PID_increment(float k_p, float k_i, float k_d, float max_output, float max_ki): PID(k_p, k_i, k_d, max_output, max_ki)
{
  prev_error[0] = 0.0f;
  prev_error[1] = 0.0f;
  prev_error[2] = 0.0f;
  u_k = 0.0f;
}

bool PID_increment::UpdateError(float _error)
{
	if (this->is_initialized = false)
	{
		printf("PID is not init!\n");
		return false;
	}
	else
	{
		this->prev_error[0]   = _error;
		this->p_error = _error - this->prev_error[1];
		this->i_error = _error;
		this->d_error = _error - 2.0 * (this->prev_error[1]) + this->prev_error[2];
		this->prev_error[2] = this->prev_error[1];
		this->prev_error[1] = _error;
		return true;
	}
}

float PID_increment::TotalError()
{
	float delta_u_k = Kp * p_error + Ki * i_error + Kd * d_error;
	// float delta_u_k = Kp * p_error + Ki*(1/(1+error*error/100)) * i_error + Kd * d_error;
	float u_k_max = 0.85;
	// if (fabs(this->error) < 40)
	// {
	// 	delta_u_k = 3.0*Kp * p_error + Ki * i_error + Kd * d_error;
	// 	u_k_max = 1.2;
	// }
	if (delta_u_k > u_k_max)
	{
		delta_u_k = u_k_max;
	}
	else if (delta_u_k < -1.0*u_k_max)
	{
		delta_u_k = -1.0*u_k_max;
	}
	this->u_k += delta_u_k;
	if (fabs(this->u_k) >= fabs(abs_saturation_value))
	{
		if (this->u_k > 0)
		{
			this->u_k = this->abs_saturation_value;
		}
		else if (this->u_k < 0)
		{
			this->u_k = -1.0 * fabs(this->abs_saturation_value);
		}
	}
	if ((this->prev_error[0] >0 && u_k<0) ||(this->prev_error[0]<0 && u_k>0))
	{
		u_k *= 0.8;
	}
	
	//printf("Dleta u_k : %f ,%f , %f , %f \n",delta_u_k,p_error,i_error,d_error);
	return this->u_k;
}

void PID_increment::Reset()
{
  prev_error[0] = 0.0f;
  prev_error[1] = 0.0f;
  prev_error[2] = 0.0f;
  u_k = 0.0f;
}

PID_AdamShan::PID_AdamShan() : PID()
{
	p_error = 0;
	need_twiddle = true;
	step = 1;
	val_step = 20;
	test_step = 200;

	index_param = 0;
	best_error = std::numeric_limits<double>::max();
	total_error = 0;
    // fail to make the total_error better times
    fail_counter = 0;
}

PID_AdamShan::PID_AdamShan(float k_p, float k_i, float k_d, float max_output, float max_ki): PID(k_p, k_i, k_d, max_output, max_ki)
{
	p_error = 0;
	need_twiddle = true;
	step = 1;
	val_step = 20;
	test_step = 200;

	index_param = 0;
	best_error = std::numeric_limits<double>::max();
	total_error = 0;
    // fail to make the total_error better times
    fail_counter = 0;
}

void PID_AdamShan::IndexMove()
{
    index_param++;
    if(index_param >=3)
	{
        index_param = 0;
    }
}

void PID_AdamShan::UpdateChange(float ratio)
{
    switch(index_param)
    {
    	case 0:
    	  change_kp *= ratio;
    	break;
    	case 1:
    	  change_ki *= ratio;
    	break;
    	case 2:
          change_kd *= ratio;
    	break;
    	default:
    	break;
    }	
}

void PID_AdamShan::UpdateParam(int mode)
{
	if(mode == 0)
	{
		if(index_param == 0)
		{
			Kp += change_kp;
		}
		if(index_param == 1)
		{
			Ki += change_ki;
		}
		if(index_param == 2)
		{
			Kd += change_kd;
		}
	}
	else
	{
		if(index_param == 0)
		{
			Kp -= 2*change_kp;
		}
		if(index_param == 1)
		{
			Ki -= 2*change_ki;
		}
		if(index_param == 2)
		{
			Kd -= 2*change_kd;
		}
	}
	std::cout << "P: "<< Kp<<" I: "<<Ki<<" D: "<<Kd<<std::endl;
}

bool PID_AdamShan::UpdateError(float _error)
{
	if (this->is_initialized = false)
	{
		printf("PID is not init!\n");
		return false;
	}
	else
	{
        if(step == 1){
            this->p_error = _error;
    
        }
        this->d_error = _error - this->p_error;
        this->p_error = _error;
        this->i_error += _error;
		if (fabs(this->i_error) > this->abs_windup) // paying attention to integral windup
		{
			if (this->i_error > 0)
			{
				this->i_error = this->abs_windup;
			}
			else if (this->i_error < 0)
			{
				this->i_error = -(1.0) * this->abs_windup;
			}
		}
        if(need_twiddle)
		{
            if(step % (val_step + test_step) > val_step)
		    {
                total_error += (_error * _error);
            }				
			if(step % (val_step + test_step) == 0)
			{
			    std::cout<<"==============  step "<<step<<" =============="<<std::endl;
			    //std::cout << "P: "<< Kp<<" I: "<<Ki<<" D: "<<Kd<<std::endl;
				if (step == (val_step + test_step))
				{
					if(total_error < best_error)
					{
						best_error = total_error;
					}
					UpdateParam(0);
				}
				else
				{
					if(total_error < best_error)
					{
						best_error = total_error;
						UpdateChange(1.1);
						IndexMove();
						UpdateParam(0);
						fail_counter = 0;
					}
					else if(fail_counter == 0)
					{
						UpdateParam(1);
						fail_counter++;
					}
					else
					{
						UpdateParam(0);
						UpdateChange(0.9);
						IndexMove();
						UpdateParam(0);
						fail_counter = 0;
					}
				}
				std::cout << "best_error: "<< best_error<<" total_error: "<<total_error<<std::endl;
				total_error = 0;
			}
		}
		step++;		
		return true;
	}
}

float PID_AdamShan::TotalError()
{
   float output = Kp * p_error + Ki * i_error + (Kd/(1+p_error*p_error)) * d_error;
   if(output > this->abs_saturation_value)
   {
       output = this->abs_saturation_value;
   }
   if(output < -1.0*(this->abs_saturation_value))
   {
       output = -1.0*(this->abs_saturation_value);
   }
   return output;
}

void PID_AdamShan::Reset()
{
	//prev_error = 0.0f;
}