#pragma once

#include "Types.h"
#include "Macros.h"

#include <utility> //for pair

namespace vdo {

class DataProvider {

public:
    VDO_POINTER_TYPEDEFS(DataProvider);

    using Inputs = std::pair<InputPacket, GroundTruthInputPacket>;
    using InputsVector = std::vector<Inputs>;


    DataProvider() = default;
    virtual ~DataProvider() = default;

    virtual size_t next(Inputs& input);
    virtual size_t next(InputPacket& input_packet, GroundTruthInputPacket::Optional ground_truth) = 0;
    virtual size_t size() const = 0;

};


class KittiSequenceDataProvider : public DataProvider {

public:
    KittiSequenceDataProvider(const std::string& path_to_sequence_);

    size_t next(InputPacket& input_packet, GroundTruthInputPacket::Optional ground_truth) override;
    size_t size() const override;

private:
    bool loadData(const std::string& path_to_sequence, InputsVector& inputs_vector);
    void loadSemanticMask(const std::string& strFilenamesMask, cv::Mat& mask);

    InputsVector data;
    size_t index = 0;


};

}