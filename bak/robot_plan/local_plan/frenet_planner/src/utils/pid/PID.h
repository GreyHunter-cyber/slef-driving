#ifndef _PID_H
#define _PID_H

#include <string>
#include <vector>

# define MAX_KI 10.0 
# define MAX_OUTPUT 30.0

class PID
{
protected:
  /*
  * Errors
  */
  float p_error;
  float i_error;
  float d_error;

  /*
  * Coefficients
  */ 
  float Kp;
  float Ki;
  float Kd;
  /*
  * Change of Coefficients
  */ 
  float change_kp;
  float change_ki; 
  float change_kd;

  /*
  * 功能性变量 FUNCTIONALRRY
  */
  float abs_saturation_value;
  float abs_windup;

public:
  PID();
  PID(float k_p, float k_i, float k_d, float max_output = MAX_OUTPUT, float max_ki = MAX_KI);
  virtual ~PID(){}

  /*
  * Initialize PID.
  */
  bool is_initialized;
  
  void Init(float k_p, float k_i, float k_d, float max_output = MAX_OUTPUT, float max_ki = MAX_KI);

  void Init(const char* filename);

public:
  /*
  * Update the PID error variables given cross track error.
  */
  virtual bool UpdateError(float _error) = 0;

  /*
  * Calculate the total PID error.
  */
  virtual float TotalError() = 0;

  virtual void Reset() = 0;

};

///////////////////////////////////////基础版PID//////////////////////////////////////////
class PID_basic : public PID
{
private:
  float prev_error; //上一次输入的error值

public:
  PID_basic();
  PID_basic(float k_p, float k_i, float k_d, float max_output = MAX_OUTPUT, float max_ki = MAX_KI);
  virtual ~PID_basic(){}

public:
  bool UpdateError(float _error);
  float TotalError();
  void Reset();
};

//////////////////////////////////////增量式PID////////////////////////////////////////////
class PID_increment : public PID
{
private:
  float prev_error[3]; //输入prev_error的buf，prev_error[0]表示最新值，prev_error[2]表示最旧值 
  float u_k;      //output value of the sequence k

public:
  PID_increment();
  PID_increment(float k_p, float k_i, float k_d, float max_output = MAX_OUTPUT, float max_ki = MAX_KI);
  virtual ~PID_increment(){}

public:
  bool UpdateError(float _error);
  float TotalError();
  void Reset();
};

class PID_AdamShan: public PID
{ 
private: 
  int step; 

  float best_error; 
  float total_error; 
  int index_param; 
  int val_step; 
  int test_step; 
  int fail_counter; 
  bool need_twiddle;

  void IndexMove();
  void UpdateChange(float ratio);
  void UpdateParam(int mode);

public: 
  PID_AdamShan(); 
  PID_AdamShan(float k_p, float k_i, float k_d, float max_output = MAX_OUTPUT, float max_ki = MAX_KI);
  virtual ~PID_AdamShan(){}; 

  bool UpdateError(float _error); 
  float TotalError(); 
  void Reset();
};
#endif