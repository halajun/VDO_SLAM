#!/bin/bash
###################################################################
# Fill the variables below

PARAM_PATH="../params"
OUTPUT_PATH="../output_logs"


if [ $(basename $(pwd)) != "scripts" ] 
then
    echo "Bash script should be run from VDO_SLAM/scripts folder."
    exit 0
fi




######################### Internal #############################
RUN_TEST_ONLY=0
LOG_OUTPUT=0
PARAM_FILE=""
DATASET_PATH=""

TEST_DATA_PATH="../test/data"
BUILD_PATH="../build"

PARAM_PATH="$(cd "$(dirname $PARAM_PATH)"; pwd)/$(basename $PARAM_PATH)"
OUTPUT_PATH="$(cd "$(dirname $OUTPUT_PATH)"; pwd)/$(basename $OUTPUT_PATH)"
TEST_DATA_PATH="$(cd "$(dirname $TEST_DATA_PATH)"; pwd)/$(basename $TEST_DATA_PATH)"
BUILD_PATH="$(cd "$(dirname $BUILD_PATH)"; pwd)/$(basename $BUILD_PATH)"


# Parse Options.
if [ $# -eq 0 ]; then
  # If there is no options tell user what are the values we are using.
  echo "Using dataset at path: $DATASET_PATH"
else
  # Parse all the options.
  while [ -n "$1" ]; do # while loop starts
      case "$1" in
        # Option -p, provides path to dataset.
      -p) PARAM_FILE=$2
          shift ;;
      -d) DATASET_PATH=$2
          shift ;;
      -t) RUN_TEST_ONLY=1
           echo "Running VDO-SLAM tests" ;;
      -log) LOG_OUTPUT=1
           echo "Logging output!";;
      --)
          shift # The double dash which separates options from parameters
          break
          ;; # Exit the loop using break command
      *) echo "Option $1 not recognized" ;;
      esac
      shift
  done
fi

# Change directory to parent path, in order to make this script
# independent of where we call it from.
cd $BUILD_PATH

FULL_PARAM_FILE=$PARAM_PATH/$PARAM_FILE
DATASET_PATH="$(cd "$(dirname $DATASET_PATH)"; pwd)/$(basename $DATASET_PATH)"

echo "Using param file at file: $FULL_PARAM_FILE"
echo "Using dataset at path: $DATASET_PATH"


if [ RUN_TEST_ONLY == 1 ]
then
    ./testVdoSlam \
        --test_data_path="$TEST_DATA_PATH" \
        --output_path="$OUTPUT_PATH" \
        --logtostderr=1 \
        --colorlogtostderr=1 \
        --log_prefix=1 \
        --v=2
else
    ./vdo_slam_kitti "$FULL_PARAM_FILE" "$DATASET_PATH" \
        --output_path="$OUTPUT_PATH" \
        --logtostderr=1 \
        --colorlogtostderr=1 \
        --log_prefix=1 \
        --v=7
fi