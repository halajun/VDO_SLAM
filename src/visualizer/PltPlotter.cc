#include "visualizer/PltPlotter.h"
#include "Map.h"
#include "Converter.h"
#include <glog/logging.h>


#include <matplotlibcpp.h>
namespace plt = matplotlibcpp;

using namespace VDO_SLAM;

std::map<std::string, Plotter::PlotData> Plotter::plots{};
int Plotter::figure = 1;


void Plotter::initPlot(const PlotInfo& plot_info) {
    if (plots.find(plot_info.title) == plots.end()) {
        LOG(INFO) << "Making new plot for: " << plot_info.title;
        PlotData plot_data;
        plot_data.info = plot_info;
        plot_data.data = std::vector<double>();
        plots[plot_info.title] = plot_data;
    }
}

void Plotter::addData(const std::string& title, const std::vector<double>& data) {
    if(!checkPlot(title)) return;

    plots[title].data = data;
}


void Plotter::appendData(const std::string& title, double data) {
    if(!checkPlot(title)) return;

    plots[title].data.push_back(data);
}

void Plotter::makePlots(const std::string& path) {
    LOG(INFO) << "Making " << plots.size() << " plots at path - " << path;
    for(const auto& p : plots) {
        const std::string& title = p.first;
        Plotter::makePlot(path, title, Plotter::figure);

        // const PlotData& plot_data = plots[title];
        // const PlotInfo& plot_info = plot_data.info;
        // const std::vector<double>& data = plot_data.data;
        // std::vector<double> x_data(data.size());
        // for(size_t i = 0; i < x_data.size(); i++) {
        //     x_data.at(i) = static_cast<double>(i + 1);
        //     LOG(INFO) << x_data[i] << " " << data[i];
        // }

        // LOG(INFO) << "Saving plot - " << plot_info.title;
        // LOG(INFO) << data.size() << " " << x_data.size();
        // plt::figure(Plotter::figure);
        // plt::title(plot_info.title);
        // plt::xlabel(plot_info.x_label);
        // plt::ylabel(plot_info.y_label);
        // plt::plot(x_data, data);
        // plt::save(path + title + ".png");

        Plotter::figure++;
    }
}

void Plotter::makePlot(const std::string& path, const std::string& title, int fig) {
    const PlotData& plot_data = plots[title];
    const PlotInfo& plot_info = plot_data.info;
    const std::vector<double>& data = plot_data.data;
    std::vector<double> x_data(data.size());
    for(size_t i = 0; i < x_data.size(); i++) {
        x_data[i] = i + 1;
    }

    LOG(INFO) << "Saving plot - " << plot_info.title;
    // LOG(INFO) << data.size() << " " << x_data.size();
    plt::figure(fig);
    plt::title(plot_info.title);
    plt::xlabel(plot_info.x_label);
    plt::ylabel(plot_info.y_label);
    plt::plot(x_data, data);
    plt::save(path + title + ".png");
    
}

void Plotter::drawDynamicSize(const std::map<int, std::vector<std::vector<gtsam::Key>>>& dynmic_motions, 
        const std::map<int, int>& when_added,
        const std::string& path) {
    plt::figure(Plotter::figure);
    plt::title("Values in system per dynamic object");
    plt::xlabel("n frames");
    plt::ylabel("# values");


    LOG(INFO) << "Plotting for n dynamic motions - " << dynmic_motions.size();
    for(const auto& e : dynmic_motions) {
        int total = 0;
        int index = when_added.at(e.first);
        std::vector<int> num_vars;
        std::vector<int> x;
        for(const auto& f : e.second) {
            
            //only add if non zero (ie. the object actually appeared in this frame)
            if(f.size() > 0) {
                total += f.size();
                num_vars.push_back(total);                
                x.push_back(index);
            }
            //  LOG(INFO) << f.size() << " plot for obj " << e.first ;
            
            index++;
        }

        LOG(INFO) << "Plotting for obj " << e.first;
        plt::named_plot("Obj: " + std::to_string(e.first), x, num_vars);        
    }

    // Enable legend.
    plt::legend();

    plt::save(path + "object_variables.png");
    Plotter::figure++;
}




void Plotter::PlotMetricError(Map* map, int max_id,  const std::string& path) {

    const std::vector<cv::Mat> &CamPose = map->vmCameraPose;
    const std::vector<std::vector<cv::Mat>>& RigMot = map->vmRigidMotion;
    const std::vector<std::vector<cv::Mat>>& ObjPosePre = map->vmObjPosePre;
    const std::vector<cv::Mat> &CamPose_gt = map->vmCameraPose_GT;
    const std::vector<std::vector<cv::Mat>>& RigMot_gt = map->vmRigidMotion_GT;
    const std::vector<std::vector<bool>>& ObjStat = map->vbObjStat;

    const std::vector<std::vector<int>>& ObjLabel = map->vnRMLabel;

    //sanity check that we have object motions - 1 to frames
    CHECK_EQ(CamPose.size(), RigMot.size() + 1);
        
    bool bRMSError = false;

    std::vector<double> x;
    std::vector<double> t_camera_error;
    std::vector<double> r_camera_error;

    std::vector<std::vector<double>> t_object_errors(max_id-1);
    std::vector<std::vector<double>> r_object_errors(max_id-1);

    std::vector<std::vector<int>> t_object_frames(max_id-1);
    std::vector<std::vector<int>> r_object_frames(max_id-1);


    float t_sum = 0, r_sum = 0;
    for (int i = 1; i < CamPose.size(); ++i)
    {
        
        x.push_back(i);
        cv::Mat T_lc_inv = CamPose[i]*Converter::toInvMatrix(CamPose[i-1]);
        cv::Mat T_lc_gt = CamPose_gt[i-1]*Converter::toInvMatrix(CamPose_gt[i]);
        cv::Mat ate_cam = T_lc_inv*T_lc_gt;
        // cv::Mat ate_cam = CamPose[i]*Converter::toInvMatrix(CamPose_gt[i]);

        // translation
        float t_ate_cam = std::sqrt(ate_cam.at<float>(0,3)*ate_cam.at<float>(0,3) + ate_cam.at<float>(1,3)*ate_cam.at<float>(1,3) + ate_cam.at<float>(2,3)*ate_cam.at<float>(2,3));
        if (bRMSError)
            t_sum = t_sum + t_ate_cam*t_ate_cam;
        else
            t_sum = t_sum + t_ate_cam;

        // rotation
        float trace_ate = 0;
        for (int j = 0; j < 3; ++j)
        {
            if (ate_cam.at<float>(j,j)>1.0)
                trace_ate = trace_ate + 1.0-(ate_cam.at<float>(j,j)-1.0);
            else
                trace_ate = trace_ate + ate_cam.at<float>(j,j);
        }
        float r_ate_cam = acos( (trace_ate -1.0)/2.0 )*180.0/3.1415926;
        if (bRMSError)
            r_sum = r_sum + r_ate_cam*r_ate_cam;
        else
            r_sum = r_sum + r_ate_cam;

        // cout << " t: " << t_ate_cam << " R: " << r_ate_cam << endl;
        t_camera_error.push_back(static_cast<double>(t_ate_cam));
        r_camera_error.push_back(static_cast<double>(r_ate_cam));

    }
    if (bRMSError)
    {
        t_sum = std::sqrt(t_sum/(CamPose.size()-1));
        r_sum = std::sqrt(r_sum/(CamPose.size()-1));
    }
    else
    {
        t_sum = t_sum/(CamPose.size()-1);
        r_sum = r_sum/(CamPose.size()-1);
    }

    // std::cout << "average error (Camera):" << " t: " << t_sum << " R: " << r_sum << endl;

    std::vector<float> each_obj_t(max_id-1,0);
    std::vector<float> each_obj_r(max_id-1,0);
    std::vector<int> each_obj_count(max_id-1,0);

    // all motion error for OBJECTS (mean error)
    // cout << "OBJECTS:" << endl;
    float r_rpe_sum = 0, t_rpe_sum = 0, obj_count = 0;
    for (int i = 0; i < RigMot.size(); ++i)
    {
        
        
        CHECK_EQ(RigMot[i].size(), ObjLabel[i].size());
        if (RigMot[i].size()>1)
        {
            for (int j = 1; j < RigMot[i].size(); ++j)
            {
                if (!ObjStat[i][j])
                {
                    // std::cout << "(" << map->vnRMLabel[i][j] << ")" << " is a failure case." << std::endl;
                    continue;
                }

                cv::Mat RigMotBody = Converter::toInvMatrix(ObjPosePre[i][j])*RigMot[i][j]*ObjPosePre[i][j];
                cv::Mat rpe_obj = Converter::toInvMatrix(RigMotBody)*RigMot_gt[i][j];

                // translation error
                float t_rpe_obj = std::sqrt( rpe_obj.at<float>(0,3)*rpe_obj.at<float>(0,3) + rpe_obj.at<float>(1,3)*rpe_obj.at<float>(1,3) + rpe_obj.at<float>(2,3)*rpe_obj.at<float>(2,3) );
                if (bRMSError){
                    each_obj_t[map->vnRMLabel[i][j]-1] = each_obj_t[map->vnRMLabel[i][j]-1] + t_rpe_obj*t_rpe_obj;
                    t_rpe_sum = t_rpe_sum + t_rpe_obj*t_rpe_obj;
                }
                else{
                    each_obj_t[map->vnRMLabel[i][j]-1] = each_obj_t[map->vnRMLabel[i][j]-1] + t_rpe_obj;
                    t_rpe_sum = t_rpe_sum + t_rpe_obj;
                }

                // rotation error
                float trace_rpe = 0;
                for (int k = 0; k < 3; ++k)
                {
                    if (rpe_obj.at<float>(k,k)>1.0)
                        trace_rpe = trace_rpe + 1.0-(rpe_obj.at<float>(k,k)-1.0);
                    else
                        trace_rpe = trace_rpe + rpe_obj.at<float>(k,k);
                }
                float r_rpe_obj = acos( ( trace_rpe -1.0 )/2.0 )*180.0/3.1415926;
                if (bRMSError){
                    each_obj_r[map->vnRMLabel[i][j]-1] = each_obj_r[map->vnRMLabel[i][j]-1] + r_rpe_obj*r_rpe_obj;
                    r_rpe_sum = r_rpe_sum + r_rpe_obj*r_rpe_obj;
                }
                else{
                    each_obj_r[map->vnRMLabel[i][j]-1] = each_obj_r[map->vnRMLabel[i][j]-1] + r_rpe_obj;
                    r_rpe_sum = r_rpe_sum + r_rpe_obj;
                }

                // cout << "(" << map->vnRMLabel[i][j] << ")" << " t: " << t_rpe_obj << " R: " << r_rpe_obj << endl;
                obj_count++;
                each_obj_count[map->vnRMLabel[i][j]-1] = each_obj_count[map->vnRMLabel[i][j]-1] + 1;


                //add to object motion
                t_object_errors[map->vnRMLabel[i][j]-1].push_back(static_cast<double>(t_rpe_obj));
                r_object_errors[map->vnRMLabel[i][j]-1].push_back(static_cast<double>(r_rpe_obj));

                t_object_frames[map->vnRMLabel[i][j]-1].push_back(i);
                r_object_frames[map->vnRMLabel[i][j]-1].push_back(i);

            }
        }
    }
    if (bRMSError)
    {
        t_rpe_sum = std::sqrt(t_rpe_sum/obj_count);
        r_rpe_sum = std::sqrt(r_rpe_sum/obj_count);
    }
    else
    {
        t_rpe_sum = t_rpe_sum/obj_count;
        r_rpe_sum = r_rpe_sum/obj_count;
    }
    // cout << "average error (Over All Objects):" << " t: " << t_rpe_sum << " R: " << r_rpe_sum << endl;

    // show each object
    for (int i = 0; i < each_obj_count.size(); ++i)
    {
        if (bRMSError)
        {
            each_obj_t[i] = std::sqrt(each_obj_t[i]/each_obj_count[i]);
            each_obj_r[i] = std::sqrt(each_obj_r[i]/each_obj_count[i]);
        }
        else
        {
            each_obj_t[i] = each_obj_t[i]/each_obj_count[i];
            each_obj_r[i] = each_obj_r[i]/each_obj_count[i];
        }
        if (each_obj_count[i]>=3) {}
            // cout << endl << "average error of Object " << i+1 << ": " << " t: " << each_obj_t[i] << " R: " << each_obj_r[i] << " TrackCount: " << each_obj_count[i] << endl;
    }

    // cout << "=================================================" << endl;

    //now make plots
    plt::figure(Plotter::figure);
    plt::title("ATE (translation)");
    plt::xlabel("n frames");
    plt::ylabel("Translation Error (m)");
    plt::named_plot("Camera", x, t_camera_error);      

    x = std::vector<double>();
    for(size_t i = 0; i < t_object_errors.size(); i++) {

        LOG(INFO) << "Obj" << i + 1 << " - " << t_object_errors.at(i).size();
        plt::named_plot("Obj: " + std::to_string(i + 1), t_object_frames.at(i), t_object_errors.at(i));  
        x.clear();      
    }  

    // Enable legend.
    plt::legend();
    plt::save(path + "ate_translation_errors.png");

    Plotter::figure++;


}

void Plotter::PlotMotionComparison(Map* map, const std::string& path) {
    const std::vector<std::vector<cv::Mat>>& RigMot_gt = map->vmRigidMotion_GT;
    //estimates from the frontend
    const std::vector<std::vector<cv::Mat>>& RigMot = map->vmRigidMotion;

    const std::vector<std::vector<cv::Mat>>& RigMot_RF = map->vmRigidMotion_RF;

    const std::vector<std::vector<cv::Mat>>& ObjPosePre = map->vmObjPosePre;

    float r_rpe_sum = 0, t_rpe_sum = 0, obj_count = 0;

    std::vector<float> frontend_errors;
    std::vector<float> backend_errors;
    std::vector<float> frames;
    float t_rpe_sum_frontend = 0, t_rpe_sum_backend = 0;
    for (int i = 0; i < RigMot.size(); ++i)
    {
        
        
        // CHECK_EQ(RigMot[i].size(), ObjLabel[i].size());
        if (RigMot[i].size()>1)
        {
            for (int j = 1; j < RigMot[i].size(); ++j)
            {
                // if (!ObjStat[i][j])
                // {
                //     // std::cout << "(" << map->vnRMLabel[i][j] << ")" << " is a failure case." << std::endl;
                //     continue;
                // }

                cv::Mat RigMotBodyFrontEnd = Converter::toInvMatrix(ObjPosePre[i][j])*RigMot[i][j]*ObjPosePre[i][j];
                cv::Mat RigMotBodyBackEnd = Converter::toInvMatrix(ObjPosePre[i][j])*RigMot_RF[i][j]*ObjPosePre[i][j];
                cv::Mat rpe_obj_frontend = Converter::toInvMatrix(RigMotBodyFrontEnd)*RigMot_gt[i][j];
                cv::Mat rpe_obj_backend = Converter::toInvMatrix(RigMotBodyBackEnd)*RigMot_gt[i][j];

                // translation error
                float t_rpe_obj_front = std::sqrt( rpe_obj_frontend.at<float>(0,3)*rpe_obj_frontend.at<float>(0,3) + rpe_obj_frontend.at<float>(1,3)*rpe_obj_frontend.at<float>(1,3) + rpe_obj_frontend.at<float>(2,3)*rpe_obj_frontend.at<float>(2,3) );
                float t_rpe_obj_back = std::sqrt( rpe_obj_backend.at<float>(0,3)*rpe_obj_backend.at<float>(0,3) + rpe_obj_backend.at<float>(1,3)*rpe_obj_backend.at<float>(1,3) + rpe_obj_backend.at<float>(2,3)*rpe_obj_backend.at<float>(2,3) );

                // rotation error front
                float trace_rpe_front = 0, trace_rpe_back = 0;
                for (int k = 0; k < 3; ++k)
                {
                    if (rpe_obj_frontend.at<float>(k,k)>1.0)
                        trace_rpe_front = trace_rpe_front + 1.0-(rpe_obj_frontend.at<float>(k,k)-1.0);
                    else
                        trace_rpe_front = trace_rpe_front + rpe_obj_frontend.at<float>(k,k);
                }
                float r_rpe_obj_front = acos( ( trace_rpe_front -1.0 )/2.0 )*180.0/3.1415926;
                // frontend_errors.push_back(r_rpe_obj_front);
                //rotation error back
                for (int k = 0; k < 3; ++k)
                {
                    if (rpe_obj_backend.at<float>(k,k)>1.0)
                        trace_rpe_back = trace_rpe_back + 1.0-(rpe_obj_backend.at<float>(k,k)-1.0);
                    else
                        trace_rpe_back = trace_rpe_back + rpe_obj_backend.at<float>(k,k);
                }
                float r_rpe_obj_back = acos( ( trace_rpe_back -1.0 )/2.0 )*180.0/3.1415926;
                // backend_errors.push_back(r_rpe_obj_back);

                t_rpe_sum_frontend+= t_rpe_obj_front;
                t_rpe_sum_backend += t_rpe_obj_back;
                obj_count++;
            

            }
            //count the total errors
            frontend_errors.push_back(t_rpe_sum_frontend/obj_count);
            backend_errors.push_back(t_rpe_sum_backend/obj_count);
            frames.push_back(i + 1);
        }
    }

    plt::figure(Plotter::figure);
    plt::title("Motion Comparison");
    plt::xlabel("n frames");
    plt::ylabel("Translation Error (m)");
    // plt::named_plot("Camera", x, t_camera_error);      

    // for(size_t i = 0; i < t_object_errors.size(); i++) {

    //     LOG(INFO) << "Obj" << i + 1 << " - " << t_object_errors.at(i).size();
    //     plt::named_plot("Obj: " + std::to_string(i + 1), t_object_frames.at(i), t_object_errors.at(i));  
    //     x.clear();      
    // }  
    plt::named_plot("Frontend Errors", frames, frontend_errors);  
    plt::named_plot("Backend Errors", frames, backend_errors);  


    // Enable legend.
    plt::legend();
    plt::save(path + "motion_comparison.png");

    Plotter::figure++;


}


bool Plotter::checkPlot(const std::string& title) {
    if (plots.find(title) == plots.end()) {
        LOG(WARNING) << "No plot found for " << title << " - did you create the plot with initPlot()";
        return false;
    }
    return true;
}