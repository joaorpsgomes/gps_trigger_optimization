#include "../include/GPS2plane.h"

using namespace std;

GPS2plane::GPS2plane(){

}


GPS2plane::GPS2plane(double latitude, double longitude, double altitude){
	double x,y,z;
	gps2cart(&x,&y,&z,latitude,longitude,altitude);
	r_ox_=x;
	r_oy_=y;
	r_oz_=z;
	double m=sqrt(pow(r_ox_,2)+pow(r_oy_,2)+pow(r_oz_,2));
	double n_x=r_ox_/m;
	double n_y=r_oy_/m;
	double n_z=r_oz_/m;
	e_xx_=-1;
	e_xy_=0;
	e_xz_=-(n_x*e_xx_+n_y*e_xy_)/n_z;

	double m_e_x=sqrt(pow(e_xx_,2)+pow(e_xy_,2)+pow(e_xz_,2));
	e_xx_=e_xx_/m_e_x;
	e_xy_=e_xy_/m_e_x;
	e_xz_=e_xz_/m_e_x;

	e_yx_=1;
	e_yz_=e_yx_*(n_x*e_xy_-e_xx_*n_y)/(e_xz_*n_y-n_z*e_xy_);
	e_yy_=-(e_yx_*n_x+e_yz_*n_z)/n_y;

	double m_e_y=sqrt(pow(e_yx_,2)+pow(e_yy_,2)+pow(e_yz_,2));
	e_yx_=e_yx_/m_e_y;
	e_yy_=e_yy_/m_e_y;
	e_yz_=e_yz_/m_e_y;

	x=e_xx_*(r_ox_-r_ox_)+e_xy_*(r_oy_-r_oy_)+e_xz_*(r_oz_-r_oz_);
	y=e_yx_*(r_ox_-r_ox_)+e_yy_*(r_oy_-r_oy_)+e_yz_*(r_oz_-r_oz_);
	x_.push_back(x);
	y_.push_back(y);
	



	/*cout<<"m: "<< m << endl;
	cout<<"n_x: "<< n_x << endl;
	cout<<"n_y: "<< n_y << endl;
	cout<<"n_z: "<< n_z << endl;
	cout<<"e_xx_: "<< e_xx_ << endl;
	cout<<"e_xy_: "<< e_xy_ << endl;
	cout<<"e_xz_: "<< e_xz_ << endl;*/

}



void GPS2plane::gps2cart(double *x, double *y, double *z, double latitude, double longitude, double altitude){
	double phy=longitude*M_PI/180;
	double theta=(90-latitude)*M_PI/180;
	double lat_rad=latitude/M_PI*180;
	double a=6378137;
	double b=6356752.3142;
	double p=altitude+sqrt((pow((pow(a,2)*cos(lat_rad)),2)+pow((pow(b,2)*sin(lat_rad)),2))/(pow((a*cos(lat_rad)),2)+pow((b*sin(lat_rad)),2)));

	*x=p*sin(theta)*cos(phy);
	*y=p*sin(theta)*sin(phy);
	*z=p*cos(theta);
	/*
	cout<<"phy: "<< phy << endl;
	cout<<"theta: "<< theta << endl;
	cout<<"lat_rad: "<< lat_rad << endl;
	cout<<"p: "<< p << endl;
	cout<<"x: "<< *x << endl;
	cout<<"y: "<< *y << endl;
	cout<<"z: "<< *z << endl;
  */            
}

void GPS2plane::gps2planeCoordinates(double *x, double *y, double latitude, double longitude, double altitude){
	double x_aux,y_aux,z_aux;
	gps2cart(&x_aux,&y_aux,&z_aux,latitude,longitude,altitude);
	*x=e_xx_*(x_aux-r_ox_)+e_xy_*(y_aux-r_oy_)+e_xz_*(z_aux-r_oz_);
	*y=e_yx_*(x_aux-r_ox_)+e_yy_*(y_aux-r_oy_)+e_yz_*(z_aux-r_oz_);
}