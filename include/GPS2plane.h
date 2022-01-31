#include <math.h>
#include <vector>


using namespace std;

class GPS2plane{
	private:
		double r_ox_;
		double r_oy_;
		double r_oz_;
		double e_xx_;
		double e_xy_;
		double e_xz_;
		double e_yx_;
		double e_yy_;
		double e_yz_;
		vector<double> x_;
		vector<double> y_;

	public:
                GPS2plane();
		GPS2plane(double latitude, double longitude, double altitude);
                void gps2planeCoordinates(double *x, double *y, double latitude, double longitude, double altitude);
		void gps2cart(double *x, double *y, double *z, double latitude, double longitude, double altitude);
		vector<double> get_x_(){return x_;}
		vector<double> get_y_(){return y_;}
		void set_point(double x, double y);
};