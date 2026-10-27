#ifndef SMOOTHER_H_
#define SMOOTHER_H_

#include <Eigen/Core>
#include <vector>
#include "esdf_map.h"
#include "path_searching/path_simplifier.h"
#include <string>

namespace navi_planner {
class smoother{
public:
    typedef std::vector<Eigen::Vector2d> VV2D;
    smoother(){;}
    ~smoother(){;}
    VV2D smooth(VV2D path_input, 
    double lambda_1 = 0.1f, double lambda_2 = 0.1f);
    void smoother_setEnvironment(ESDF_enviroment::Ptr esdf_ptr);
    VV2D pathResampling(VV2D path_input);
    bool isCollision(Eigen::Vector2d node1, Eigen::Vector2d node2);
    void configureSimplification(const std::string &mode, double tolerance_m);
    void setSimplificationEnvironment(
        const path_searching::GridGeometry &geometry,
        path_searching::PathSimplifier::CellQuery query);
private:
    ESDF_enviroment::Ptr esdf_environment_;
    bool use_tdt_simplification_{false};
    path_searching::PathSimplifier simplifier_;
    path_searching::GridGeometry simplification_geometry_;
    path_searching::PathSimplifier::CellQuery simplification_query_;
    
};
}

#endif
