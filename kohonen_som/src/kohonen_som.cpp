#include <cassert>
#include <iostream>
#include <vector>

#include <knnl/std_headers.hpp>
#include <knnl/neural_net_headers.hpp>

#include <ros/ros.h>
#include <kohonen_som/Clustering.h>
#include <geometry_msgs/Point.h>

typedef std::vector<std::vector<double> > Matrix2d;
typedef neural_net::Cauchy_function<std::vector<double>::value_type, std::vector<double>::value_type, int> CF;
typedef neural_net::Gauss_function<std::vector<double>, std::vector<double>, int> GF;
typedef distance::Euclidean_distance_function<std::vector<double> > EDF;
typedef neural_net::Basic_neuron<CF, EDF> KNeuron;
typedef neural_net::Rectangular_container<KNeuron> KNetwork;
typedef neural_net::Wta_proportional_training_functional <std::vector<double>, double, int> WTF;
typedef neural_net::Wta_training_algorithm<KNetwork, std::vector<double>, Matrix2d::iterator, WTF> WTA;

class KohonenSOMNode
{
public:
  ros::NodeHandle nh_, pnh_;
  ros::ServiceServer server_;

  KohonenSOMNode(): nh_(), pnh_("~") {
    server_ = pnh_.advertiseService("clustering",
                                    &KohonenSOMNode::service_cb,
                                    this);
  };

  bool service_cb(kohonen_som::Clustering::Request  &req,
                  kohonen_som::Clustering::Response &res) {
    if (req.data.size() <= 0){
      ROS_ERROR("request data size must be > 0");
      return false;
    }

    CF cf((double)req.cauchy_scale_factor, (int)req.exponent);
    EDF edf;
    KNetwork knn;
    neural_net::Internal_randomize IR;
    neural_net::External_randomize ER;
    Matrix2d data;
    WTF wtf((double)req.learning_ratio_scale_factor, (int)req.exponent);
    for (int i = 0; i < req.data.size(); ++i){
      std::vector<double> d = req.data[i].data;
      data.push_back(d);
    }

    neural_net::generate_kohonen_network((int)req.dimension,
                                         (int)req.dimension,
                                         cf,
                                         edf,
                                         data,
                                         knn,
                                         IR);

    WTA train_func(wtf);
    for (int i = 0; i < req.iteration; ++i) {
      train_func(data.begin(), data.end(), &knn);
    }

    // FIXME
    neural_net::print_network_weights(std::cout, knn);

    return true;
  }

};

template <class V>
double distance(const V& a, const V& b) {
  assert(a.size() == b.size());
  double d = 0.0;
  for (int i = 0; i < a.size(); ++i){
    d += (a[i] - b[i]) * (a[i] - b[i]);
  }
  return d;
}

template <class V>
struct distance_compare {
  distance_compare(const V& ref) : ref_(ref){};
  bool operator()(const V& a, const V& b) const {
    return distance(a, ref) < distance(b, ref);
  }
  const T& ref_;
};

template <class V>
void find_nearest_indices(const V& v1, const V& v2, std::vector<int> &indices){
  for (V::iterator it = v1.begin(); it <!= v1.end(); ++it){
    V::iterator nearest = std::min_element(v2.begin(), v2.end(), distance_compare<V>(*it));
    indices.push_back

int main (int argc, char** argv)
{
  ros::init(argc, argv, "kohonen_som");
  KohonenSOMNode n;
  ros::spin();

  return 0;
}

