#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <armadillo>
using namespace arma;

double observe_Z=0.0;
double estimate_Z=0.0;
std_msgs::Float64 estimate_Z_msgs;
ros::Publisher output_pub;  
ros::Subscriber input_sub;

mat X(2,1);mat X_(2,1);
mat P(2,2);mat P_(2,2);
mat F(2,2);mat F_TRAN(2,2);
mat Q(2,2);mat FP(2,2);mat FPF_TRAN(2,2);
mat H(1,2);mat HP_(1,2);mat H_TRAN(2,1);mat P_H_TRAN(2,1);mat HP_H_TRAN(1,1);mat HP_H_TRAN_ADD_R(1,1);
mat HX_(1,1);

mat K(2,1);mat K_TERM(2,1);mat KH(2,2);
mat P_TERM(2,2);
mat R(1,1);
mat I(2,2);


//kalman filter
/*
float X[2][1];float X_[2][1];
float P[2][2];float P_[2][2];
float F[2][2];float F_TRAN[2][2];
float Q[2][2];float FP[2][2];float FPF_TRAN[2][2];
float H[1][2];float HP_[1][2];float H_TRAN[2][1];float P_H_TRAN[2][1];float HP_H_TRAN[1][1];float HP_H_TRAN_ADD_R[1][1];
float HX_[1][1];
float K[2][1];float K_TERM[2][1];float KH[2][2];
float P_TERM[2][2];
float R[1][1];
float I[2][2];
*/

void KalmanFilter(){
    X_=F*X;
    X_.print("X_=F*X: ");
    P_=F*P*F.t()+Q;
    P_.print("P_=F*P*F'+Q: ");
    K=(P_*H.t())/as_scalar(H*P_*H.t()+R);
    K.print("K: ");
    X=X_+K*(observe_Z-as_scalar(H*X_));
    X.print("X: ");
    estimate_Z=X(0,0);
  }

void KalmanFilterCallback(const std_msgs::Float64& msg){
   observe_Z=msg.data;
   KalmanFilter();  
   estimate_Z_msgs.data=estimate_Z;
   output_pub.publish(estimate_Z_msgs);
   ROS_INFO("PreValue:%f  KalmanValue %f",observe_Z,estimate_Z);
}
/*
float KalmanFilter(float observe_Z){
    float estimate_Z=0;
    //Serial.println(i);
    //X_=F*X;
    Matrix.Multiply((float*)F, (float*)X, 2, 2, 1, (float*)X_);
    //P_=F*P*F'+Q;
    Matrix.Multiply((float*)F, (float*)P, 2, 2, 2, (float*)FP);    
    Matrix.Transpose((float*)F, 2, 2, (float*)F_TRAN);
    Matrix.Multiply((float*)FP, (float*)F_TRAN, 2, 2, 2, (float*)FPF_TRAN);  
    Matrix.Add((float*) FPF_TRAN, (float*) Q, 2, 2, (float*) P_);
    //K=P_*H'/(H*P_*H'+R);
    Matrix.Transpose((float*) H, 1, 2, (float*) H_TRAN);    
    Matrix.Multiply((float*)P_, (float*)H_TRAN, 2, 2, 1, (float*)P_H_TRAN);  
    Matrix.Multiply((float*)H, (float*)P_, 1, 2, 2, (float*)HP_); 
    Matrix.Multiply((float*)HP_, (float*)H_TRAN, 1, 2, 1, (float*)HP_H_TRAN);   
    Matrix.Add((float*) HP_H_TRAN, (float*) R, 1, 1, (float*) HP_H_TRAN_ADD_R);
    K[0][0]=P_H_TRAN[0][0]/HP_H_TRAN_ADD_R[0][0];
    K[1][0]=P_H_TRAN[1][0]/HP_H_TRAN_ADD_R[0][0];
    //X=X_+K*(Z(i)-H*X_);
    Matrix.Multiply((float*)H, (float*)X_, 1, 2, 1, (float*)HX_);
    K_TERM[0][0]=K[0][0]*(observe_Z- HX_[0][0]);
    K_TERM[1][0]=K[1][0]*(observe_Z- HX_[0][0]);
    Matrix.Add((float*) X_, (float*) K_TERM, 2, 1, (float*) X);
    estimate_Z=X[0][0];
    //P=(eye(2)-K*H)*P_;
    Matrix.Multiply((float*)K, (float*)H, 2, 1, 2, (float*)KH);  
    Matrix.Subtract((float*) I, (float*) KH, 2, 2, (float*) P_TERM);  
    Matrix.Multiply((float*)P_TERM, (float*)P_, 2, 2, 2, (float*)P);
    return estimate_Z;
  
  }
*/

  
int main(int argc, char** argv)
{
  ros::init(argc, argv, "KalmanFilter");
  ros::NodeHandle nh_;
  output_pub = nh_.advertise<std_msgs::Float64>("/KalmanFilter/output", 500);
  input_sub = nh_.subscribe("/KalmanFilter/input", 1000, KalmanFilterCallback);




//Initialize
   X<<0<<endr<<0<<endr;
   P<<1<<0<<endr<<0<<1<<endr;
   F<<1<<0.5<<endr<<0<<1<<endr;
   Q<<0.00001<<0<<endr<<0<<0.00001<<endr;
   H<<1<<0<<endr;
   R<<0.1<<endr;
   I<<1<<0<<endr<<0<<1<<endr;
   X.print("X: ");
   P.print("P: ");
   F.print("F: ");
   Q.print("Q: ");
   H.print("H: ");
   R.print("R: ");
   I.print("I: ");

  ros::Rate r(100);
  while(nh_.ok())
  {
    r.sleep();
    ros::spinOnce();
  }
/*
   X[0][0]=0.0f; X[1][0]=0.0f;
   P[0][0]=1.0f; P[0][1]=0.0f; P[1][0]=0.0f; P[1][1]=1.0f;
   F[0][0]=1.0f; F[0][1]=0.5f;//sampling time is 0.1s 
   F[1][0]=0.0f; F[1][1]=1.0f;
   Q[0][0]=0.00001f; Q[0][1]=0.0f; Q[1][0]=0.0f; Q[1][1]=0.00001f;
   H[0][0]=1.0f; H[0][1]=0.0f;
   R[0][0]=0.1f;
  
   I[0][0]=1.0f;I[0][1]=0.0f;I[1][0]=0.0f;I[1][1]=1.0f;
*/
 // omega_actual_filter=KalmanFilter((float)omega_actual);

  return(0);
}

