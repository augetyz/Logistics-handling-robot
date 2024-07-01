#include "kalman.h"
#include "mpu6050.h"
#include <math.h>

//���������㷨��
short Accel[3];				//���ٶȴ�����ԭʼ����
short Gyro[3];				//������ԭʼ����
short aacx,aacy,aacz;		//���ٶȴ�����ԭʼ���� 
short gyrox,gyroy,gyroz;	//������ԭʼ���� 
float temperature;			//�������¶�����
float Accel_x;	     		//X����ٶ�ֵ�ݴ�
float Accel_y;	    		//Y����ٶ�ֵ�ݴ�
float Accel_z;	     		//Z����ٶ�ֵ�ݴ�
float Gyro_x;				//X�������������ݴ�
float Gyro_y;        		//Y�������������ݴ�
float Gyro_z;		 		//Z�������������ݴ�	
float Angle_x_temp;  		//�ɼ��ٶȼ����x��б�Ƕ�
float Angle_y_temp;  		//�ɼ��ٶȼ����y��б�Ƕ�
float Angle_X_Final; 		//X������б�Ƕ�
float Angle_Y_Final; 		//Y������б�Ƕ�

//�Ƕȼ���
void Angle_Calcu(short* Accel,short* Gyro)	 
{
	float accx,accy,accz;//������Ǽ��ٶ�ֵ
	Accel_x = Accel[0]; //x����ٶ�ֵ��ȡ
	Accel_y = Accel[1]; //y����ٶ�ֵ��ȡ
	Accel_z = Accel[2]; //z����ٶ�ֵ��ȡ
	Gyro_x = Gyro[0];  //x��������ֵ��ȡ
	Gyro_y = Gyro[1];  //y��������ֵ��ȡ
	Gyro_z = Gyro[2];  //z��������ֵ��ȡ
	//temperature = 36.53+((double)GetData(TEMP_OUT_H))/340.0;//�õ��¶�ֵ
	aacx= Accel_x;//x����ٶ�ֵ�ݴ�
	aacy = Accel_y;//y����ٶ�ֵ�ݴ�
	aacz = Accel_z;//z����ٶ�ֵ�ݴ�
	gyrox = Gyro_x;//x��������ֵ�ݴ�
	gyroy = Gyro_y;//y��������ֵ�ݴ�
	gyroz = Gyro_z;//z��������ֵ�ݴ� 
	
	//�Ǽ��ٶ�ԭʼֵ�������	
	//���ٶȴ��������üĴ���0X1C��д��0x01,���÷�ΧΪ��2g�������ϵ��2^16/4 = 16384LSB/g
	if(Accel_x<32764) accx=Accel_x/16384;//����x����ٶ�
	else              accx=1-(Accel_x-49152)/16384;
	if(Accel_y<32764) accy=Accel_y/16384;//����y����ٶ�
	else              accy=1-(Accel_y-49152)/16384;
	if(Accel_z<32764) accz=Accel_z/16384;//����z����ٶ�
	else              accz=(Accel_z-49152)/16384;
	//���ٶȷ����й�ʽ�����������ˮƽ������ϵ֮��ļн�
	Angle_x_temp=(atan(accy/accz))*180/3.14;
	Angle_y_temp=(atan(accx/accz))*180/3.14;
	//�жϼ����Ƕȵ�������											
	if(Accel_x<32764) Angle_y_temp = +Angle_y_temp;
	if(Accel_x>32764) Angle_y_temp = -Angle_y_temp;
	if(Accel_y<32764) Angle_x_temp = +Angle_x_temp;
	if(Accel_y>32764) Angle_x_temp = -Angle_x_temp;
	
	//���ٶ�ԭʼֵ�������
	//���������üĴ���0X1B��д��0x18�����÷�ΧΪ2000deg/s�������ϵ��2^16/4000=16.4LSB/(��/S)
	////������ٶ�
	if(Gyro_x<32768) Gyro_x=-(Gyro_x/16.4);
	if(Gyro_x>32768) Gyro_x=+(65535-Gyro_x)/16.4;
	if(Gyro_y<32768) Gyro_y=-(Gyro_y/16.4);
	if(Gyro_y>32768) Gyro_y=+(65535-Gyro_y)/16.4;
	if(Gyro_z<32768) Gyro_z=-(Gyro_z/16.4);
	if(Gyro_z>32768) Gyro_z=+(65535-Gyro_z)/16.4;
	
	Kalman_Filter_X(Angle_x_temp,Gyro_x);  //�������˲�����X���
	Kalman_Filter_Y(Angle_y_temp,Gyro_y);  //�������˲�����Y���															  
} 


//����������		
float Q_angle = 0.001;		//�Ƕ��������Ŷȣ��Ƕ�������Э����
float Q_gyro  = 0.003;		//���ٶ��������Ŷȣ����ٶ�������Э����  
float R_angle = 0.5;		//���ٶȼƲ���������Э����
float dt      = 0.02;		//�˲��㷨�������ڣ��ɶ�ʱ����ʱ20ms
char  C_0     = 1;			//H����ֵ
float Q_bias, Angle_err;	//Q_bias:�����ǵ�ƫ��  Angle_err:�Ƕ�ƫ�� 
float PCt_0, PCt_1, E;		//����Ĺ�����
float K_0, K_1, t_0, t_1;	//����������  K_0:���ڼ������Ź���ֵ  K_1:���ڼ������Ź���ֵ��ƫ�� t_0/1:�м����
float P[4] ={0,0,0,0};	//����Э��������΢�־����м����
float PP[2][2] = { { 1, 0 },{ 0, 1 } };//����Э�������P

void Kalman_Filter_X(float Accel,float Gyro) //����������		
{
	//����һ���������
	//��ʽ��X(k|k-1) = AX(k-1|k-1) + BU(k)
	//X = (Angle,Q_bias)
	//A(1,1) = 1,A(1,2) = -dt
	//A(2,1) = 0,A(2,2) = 1
	Angle_X_Final += (Gyro - Q_bias) * dt; //״̬����,�Ƕ�ֵ�����ϴ����ŽǶȼӽ��ٶȼ���Ư�����
	
	//��������������Э��������΢�־���
	//��ʽ��P(k|k-1)=AP(k-1|k-1)A^T + Q 
	//Q(1,1) = cov(Angle,Angle)	Q(1,2) = cov(Q_bias,Angle)
	//Q(2,1) = cov(Angle,Q_bias)	Q(2,2) = cov(Q_bias,Q_bias)
	P[0]= Q_angle - PP[0][1] - PP[1][0];
	P[1]= -PP[1][1];// ����������Э����
	P[2]= -PP[1][1];
	P[3]= Q_gyro;
	PP[0][0] += P[0] * dt;   
	PP[0][1] += P[1] * dt;   
	PP[1][0] += P[2] * dt;
	PP[1][1] += P[3] * dt;	
	Angle_err = Accel - Angle_X_Final;	//Z(k)������� ����Ƕ�ƫ��
	
	//�����������㿨��������
	//��ʽ��Kg(k)= P(k|k-1)H^T/(HP(k|k-1)H^T+R)
	//Kg = (K_0,K_1) ��ӦAngle,Q_bias����
	//H = (1,0)	����z=HX+v���z:Accel
	PCt_0 = C_0 * PP[0][0];
	PCt_1 = C_0 * PP[1][0];
	E = R_angle + C_0 * PCt_0;
	K_0 = PCt_0 / E;
	K_1 = PCt_1 / E;
	
	//�����ģ�����������Э����
	//��ʽ��P(k|k)=(I-Kg(k)H)P(k|k-1)
	//Ҳ��дΪ��P(k|k)=P(k|k-1)-Kg(k)HP(k|k-1)
	t_0 = PCt_0;
	t_1 = C_0 * PP[0][1];
	PP[0][0] -= K_0 * t_0;		
	PP[0][1] -= K_0 * t_1;
	PP[1][0] -= K_1 * t_0;
	PP[1][1] -= K_1 * t_1;
	
	//�����壬�������Ž��ٶ�ֵ
	//��ʽ��X(k|k)= X(k|k-1)+Kg(k)(Z(k)-X(k|k-1))
	Angle_X_Final += K_0 * Angle_err;	 //������ƣ��������Ź���ֵ
	Q_bias        += K_1 * Angle_err;	 //������ƣ��������Ź���ֵƫ��
	Gyro_x         = Gyro - Q_bias;	 
}

void Kalman_Filter_Y(float Accel,float Gyro) 		
{
	Angle_Y_Final += (Gyro - Q_bias) * dt;
	P[0]=Q_angle - PP[0][1] - PP[1][0]; 
	P[1]=-PP[1][1];
	P[2]=-PP[1][1];
	P[3]=Q_gyro;	
	PP[0][0] += P[0] * dt; 
	PP[0][1] += P[1] * dt;  
	PP[1][0] += P[2] * dt;
	PP[1][1] += P[3] * dt;	
	Angle_err = Accel - Angle_Y_Final;		
	PCt_0 = C_0 * PP[0][0];
	PCt_1 = C_0 * PP[1][0];	
	E = R_angle + C_0 * PCt_0;	
	K_0 = PCt_0 / E;
	K_1 = PCt_1 / E;	
	t_0 = PCt_0;
	t_1 = C_0 * PP[0][1];
	PP[0][0] -= K_0 * t_0;		
	PP[0][1] -= K_0 * t_1;
	PP[1][0] -= K_1 * t_0;
	PP[1][1] -= K_1 * t_1;		
	Angle_Y_Final	+= K_0 * Angle_err;
	Q_bias	+= K_1 * Angle_err;	 
	Gyro_y   = Gyro - Q_bias;	 
}

