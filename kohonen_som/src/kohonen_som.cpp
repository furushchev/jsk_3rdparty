#include <iostream>
#include <vector>

#include <knnl/std_headers.hpp>
#include <knnl/neural_net_headers.hpp>

#include <ros/ros.h>
#include <kohonen_som/Clustering.h>

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
    CF cf((double)req.cauchy_scale_factor, (int)req.exponent);
    EDF edf;
    KNetwork knn;
    neural_net::Internal_randomize IR;
    neural_net::External_randomize ER;
    std::vector<std::vector<double> > data;
    WTF wtf((double)req.learning_ratio_scale_factor, (int)req.exponent);
    for (int i = 0; i < req.data.size(); ++i){
      std::vector<double> d = req.data[i].data;
      data.push_back(d);
    }

    neural_net::generate_kohonen_network((int)req.dimension,
                                         (int)req.data.size(),
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


int main (int argc, char** argv)
{
  ros::init(argc, argv, "kohonen_som");
  KohonenSOMNode n;
  ros::spin();

  return 0;
}

