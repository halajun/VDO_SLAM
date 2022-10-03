#include "visualizer/PltPlotter.h"
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

    // LOG(INFO) << "Saving plot - " << plot_info.title;
    // LOG(INFO) << data.size() << " " << x_data.size();
    plt::figure(fig);
    plt::title(plot_info.title);
    plt::xlabel(plot_info.x_label);
    plt::ylabel(plot_info.y_label);
    plt::plot(x_data, data);
    plt::save(path + title + ".png");
    
}

void Plotter::drawDynamicSize(const std::map<int, std::vector<std::vector<gtsam::Key>>>& dynmic_motions, const std::string& path) {
    plt::figure(Plotter::figure);
    plt::title("Values in system per dynamic object");
    plt::xlabel("n frames");
    plt::ylabel("# values");

    std::vector<double> x;
    //assume we add (even an emtpy) vector at each update so the number of rows will be the number of frames
    //for each object. 
    for(int i = 0; i < dynmic_motions.at(1).size(); i++) {
        x.push_back(i+1);
    }
    LOG(INFO) << "Plotting for n dynamic motions - " << dynmic_motions.size();
    for(const auto& e : dynmic_motions) {
        int total = 0;
        std::vector<int> num_vars;
        for(const auto& f : e.second) {
            total += f.size();
            num_vars.push_back(total);
        }

        LOG(INFO) << "Plotting for obj " << e.first;
        plt::named_plot("Obj: " + std::to_string(e.first), x, num_vars);        
    }

    // Enable legend.
    plt::legend();

    plt::save(path + "object_variables.png");
    Plotter::figure++;
}

bool Plotter::checkPlot(const std::string& title) {
    if (plots.find(title) == plots.end()) {
        LOG(WARNING) << "No plot found for " << title << " - did you create the plot with initPlot()";
        return false;
    }
    return true;
}