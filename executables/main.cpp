#include "../src/defs.hpp"

#include <boost/tuple/tuple.hpp>


using namespace ProbabilisticModelling;
using namespace std;


void colorPoint(RGBImage& img, int row , int col){
  int v = 255;
  img.at<cv::Vec3b>(row,col) = cv::Vec3b(v,v,v);
}


int main(int argc, char** argv){

//  std::cout << "debug\n";
// std::cout << "PATH=" << getenv("PATH") << std::endl;
  int rows=80;
  int cols=80;
  int num_pos=500;

  Vector2iVector pointPositions(num_pos);

 // std::cout << "debug\n";
  for (size_t i=0; i<pointPositions.size(); i++){
    pointPositions[i]=Eigen::Vector2i(rows*drand48(), cols*drand48());
  }

  RGBImage shown_image;
  shown_image.create(rows, cols);
  shown_image=cv::Vec3b(255,0,0);
//
  cv::namedWindow("distance map");
    //test boost
    std::cout << "debug\n" << endl;
    std::vector<boost::tuple<double, double, double, double> > pts_A;
    pts_A.push_back(boost::make_tuple(
                                      1,
                                      5,
                                      1,
                                      9
                                      ));

  for (size_t j=0; j<pointPositions.size(); j++){
    const Eigen::Vector2i& point=pointPositions[j];
    int r=point.x();
    int c=point.y();
    colorPoint(shown_image, r, c);
    cv::imshow("distance map", shown_image);
    std::cout <<  j  << endl;
    cv::waitKey(1);
  }
}
