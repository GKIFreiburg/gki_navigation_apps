#include "calibrationmain.h"
#include "ui_calibrationmain.h"


CalibrationMain::CalibrationMain(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::CalibrationMain),
    ac("calibrate_twist", true)
{
    ui->setupUi(this);
    count = 0;
    listModel = new QStandardItemModel();

    // read all necessary parameters
    keepFile = true;
    ros::NodeHandle nhPriv("~");
    nhPriv.getParamCached("filename", filename);
    nhPriv.getParamCached("yamlname", yamlname);
    nhPriv.getParamCached("keepFile", keepFile);

    ROS_INFO("Waiting for action server to start");
    // wait for the action server to start
    ac.waitForServer(); //will wait for infinite time

    ROS_INFO("Action server started successfully");

    // Writing the result in a textfile
    string date = to_simple_string(ros::Time::now().toBoost());
    ifstream f(filename.c_str());
    if (f.good() && keepFile) // checking for existence of the file
    {
        ROS_INFO("File already present");
        //ofstream resultFile(filename.c_str(), ios::out | ios::app);
        ofstream resultFile("result.txt", ios::out | ios::app);
        resultFile << "New session started ("<<date <<")\n\n";
        resultFile.close();
    }
    else
    {
        ROS_INFO("File newly created");
        //ofstream resultFile(filename.c_str(), ios::out);
        ofstream resultFile("result.txt", ios::out);
        resultFile << "#This is the result file (created on " <<date <<")\n\n";
        resultFile.close();
    }
    f.close();

    // Write / create YAML file
    ifstream yamlfile(yamlname.c_str());
    if (yamlfile.good() && keepFile) // checking for existence of the file
    {
        ROS_INFO("YAML-File already present");
        //ofstream resultFile(filename.c_str(), ios::out | ios::app);
        ofstream calFile("vxvrCalibration.yaml", ios::out | ios::app);
        calFile << "\n#New session started ("<<date <<")\n";
        calFile.close();
    }
    else
    {
        ROS_INFO("File newly created");
        //ofstream resultFile(filename.c_str(), ios::out);
        ofstream calFile("vxvrCalibration.yaml", ios::out);
        calFile << "#This is the result file (created on " <<date <<")\n\n";
        calFile.close();
    }
    yamlfile.close();

   listener = new tf::TransformListener();
}

CalibrationMain::~CalibrationMain()
{
    delete ui;
}

void CalibrationMain::on_buttonCancel_clicked()
{
    ac.cancelGoal();
}

void CalibrationMain::on_buttonClear_clicked()
{
    listModel->clear();
    ui->listResult->setModel(listModel);
}

void CalibrationMain::on_buttonStart_clicked()
{
    double xSpeed = ui->SpinBoxXSpeed->value();
    double rotSpeed = ui->SpinBoxRotSpeed->value();
    double time = ui->SpinBoxRotSpeed_2->value();

    send_goal(xSpeed, rotSpeed, time);
}

void CalibrationMain::on_buttonContStart_clicked()
{
    startContinuousRun();
}

void CalibrationMain::on_buttonContCancel_clicked()
{
    return;
}


void CalibrationMain::on_buttonSendHome_clicked()
{
    // test only!!
    bool achieved = false;
    achieved = sendHomePositionGoal();
    if(achieved)
    {
        ROS_INFO("Reached home");
    }
    else
    {
        ROS_INFO("Did not reach home");
    }
}


void CalibrationMain::send_goal(double vx, double vrot, double time)
{
    // send a goal to the action
    calibrate_twist::CalibrateGoal goal;
    goal.duration = ros::Duration(time);
    goal.twist_goal.angular.z = vrot;
    goal.twist_goal.angular.x = 0;
    goal.twist_goal.angular.y = 0;
    goal.twist_goal.linear.x = vx;
    goal.twist_goal.linear.y = 0;
    goal.twist_goal.linear.z = 0;


//    if(ac.isServerConnected())
//    {
        ac.sendGoal(goal,
                boost::bind(&CalibrationMain::doneCB, this, _1, _2),
                Client::SimpleActiveCallback(),
                Client::SimpleFeedbackCallback());
        currentGoal = goal;
/*    }
    else
    {
        ROS_INFO("Server not connected");
        ui->labelStatus->setText("Server not connected");
        ac.waitForServer(ros::Duration(10));
        if(ac.isServerConnected())
        {
            ROS_INFO("reconnected");
        }
        else
        {
            ROS_INFO("disconnected");
        }
    }
*/
}

void CalibrationMain::doneCB(const actionlib::SimpleClientGoalState& goal, const calibrate_twist::CalibrateResultConstPtr& result)
{
    actionlib::SimpleClientGoalState state = ac.getState();
    ROS_INFO("Action finished: %s",state.toString().c_str());
    QString text = QString::fromStdString(state.toString());
    ui->labelStatus->setText("State: " + text);
    goalSucceeded = (goal == actionlib::SimpleClientGoalState::SUCCEEDED); // storing if goal succeeded

    if(state == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      ROS_INFO("state is succeeded");

      calibrate_twist::CalibrateResultConstPtr result = ac.getResult();
      // store goal and result in a vector
      goals.push_back(currentGoal);
      results.push_back(*result);

      // Writing the result in a textfile
      printToFile(result);

      QString myString = "vx: " + QString::number(result->calibrated_result.twist.linear.x) + ", " +
                         "vrot: " + QString::number(result->calibrated_result.twist.angular.z);
      QStandardItem* items = new QStandardItem(myString);
      listModel->appendRow(items);
      ui->listResult->setModel(listModel);
    }
}

void CalibrationMain::printToFile(const calibrate_twist::CalibrateResultConstPtr& result)
{
    double calibrate_vx = (currentGoal.twist_goal.linear.x/result->calibrated_result.twist.linear.x);
    double calibrate_vrot = (currentGoal.twist_goal.angular.z/result->calibrated_result.twist.angular.z);
    // calibration value can never be 0 -> only happens if 0 is divided by 0, but the correct calibration value then is 1
    if (compareDouble (calibrate_vrot,0.0))
        calibrate_vx = 1;
    if (compareDouble (calibrate_vrot,0.0))
        calibrate_vrot = 1;

    string date = to_simple_string(ros::Time::now().toBoost());
    //ofstream resultFile(filename.c_str(), ios::out | ios::app);
    ofstream resultFile("result.txt", ios::out | ios::app);
    resultFile << "Calibration run on " <<date;
    resultFile << "\n\tGoal: vx: " << currentGoal.twist_goal.linear.x <<" vrot: " <<currentGoal.twist_goal.angular.z <<"(Duration: " <<currentGoal.duration.toSec() <<"s)" // goal
               << "\n\tOdoResult: vx: " <<result->odo_result.twist.linear.x <<" vrot: " <<result->odo_result.twist.angular.z // result
               <<"\n\tResult: vx: " <<result->calibrated_result.twist.linear.x <<" vrot: " <<result->calibrated_result.twist.angular.z // result
               <<"\n\tCalibration value: vx: " <<calibrate_vx <<" vrot: " <<calibrate_vrot <<"\n\n"; //calibration value
    resultFile.close();
    ROS_INFO("Result written to file");
}
/* Old format!
void CalibrationMain::printStoreToYAML()
{
    ROS_ASSERT_MSG(results.size() == goals.size(), "Results and Goals differ in size!");

    ofstream calFile("vxvrCalibration.yaml", ios::out | ios::app);
    calFile << "\nCalibration:\n";

    unsigned int num = goals.size();
    for (unsigned int i = 0; i<num;i++)
    {
        double calibrate_vx = (goals[i].twist_goal.linear.x/results[i].calibrated_result.twist.linear.x);
        double calibrate_vrot = (goals[i].twist_goal.angular.z/results[i].calibrated_result.twist.angular.z);
        // calibration value can never be 0 -> only happens if 0 is divided by 0, but the correct calibration value then is 1
        if (compareDouble (calibrate_vrot,0.0))
            calibrate_vx = 1;
        if (compareDouble (calibrate_vrot,0.0))
            calibrate_vrot = 1;

        calFile <<goals[i].twist_goal.linear.x<<"\\"<<goals[i].twist_goal.angular.z<<": ";
        calFile <<"{vx: "<<calibrate_vx <<", vr: " <<calibrate_vrot <<"}\n";
    }
        calFile.close();
        ROS_INFO("%i Calibration entries written to YAML", num);
}
*/


void CalibrationMain::printStoreToYAML()
{
    ROS_ASSERT_MSG(results.size() == goals.size(), "Results and Goals differ in size!");
    double xSpeedInc = ui->SpinBoxContXSpeed->value();
    double rotSpeedInc = ui->SpinBoxContRotSpeed->value();

    ofstream calFile("vxvrCalibration.yaml", ios::out | ios::app);
    calFile << "\n#Calibration:\n";

    calFile << "\nvxvrInd: ";
    if(vxvrInd)
    {
        calFile << "true\n";

        unsigned int num = goals.size();
        calFile << "tv:\n  resolution: " <<xSpeedInc;
        calFile << "\nstart_value: " <<goals[0].twist_goal.linear.x;
        calFile << "\n  data:\n";
        for (unsigned int i = 0; i<num;i++)
        {
            double calibrate_vx = (goals[i].twist_goal.linear.x/results[i].calibrated_result.twist.linear.x);
            if (compareDouble (calibrate_vx,0.0))
                calibrate_vx = 1;
            calFile <<"  -  " <<calibrate_vx;
        }

        calFile << "\nrv:\n  resolution: " <<rotSpeedInc;
        calFile << "\n  start_value: " <<goals[0].twist_goal.angular.z;
        calFile << "\n  data:\n";
        for (unsigned int i = 0; i<num;i++)
        {
            double calibrate_vrot = (goals[i].twist_goal.angular.z/results[i].calibrated_result.twist.angular.z);
            if (compareDouble (calibrate_vrot,0.0))
                calibrate_vrot = 1;
            calFile <<"  -  " <<calibrate_vrot;
        }
        calFile.close();
        ROS_INFO("%i Calibration entries written to YAML", num);
    }
    else
    {
        // vx and vr not independent: there are "iterations"-count of rot-vals for each trans-val
        // rv part contains several data entries: data1, data2 ...data"iterations"

        calFile << "false\n";

        unsigned int num = goals.size();
        calFile << "tv:\n  resolution: " <<xSpeedInc;
        calFile << "\nstart_value: " <<goals[0].twist_goal.linear.x;
        calFile << "\n  data:\n";
        for (unsigned int i = 0; i<num;i+iterations)
        {
            double calibrate_vx = (goals[i].twist_goal.linear.x/results[i].calibrated_result.twist.linear.x);
            if (compareDouble (calibrate_vx,0.0))
                calibrate_vx = 1;
            calFile <<"  -  " <<calibrate_vx;
        }

        calFile << "\nrv:\n  resolution: " <<rotSpeedInc;
        calFile << "\n  start_value: " <<goals[0].twist_goal.angular.z;
        for(unsigned int j = 1; j<=iterations; j++)
        {
            calFile << "\n  data" <<j <<":\n";
            for (unsigned int i = 0+j*iterations; i<j*iterations;i++)
            {
                double calibrate_vrot = (goals[i].twist_goal.angular.z/results[i].calibrated_result.twist.angular.z);
                if (compareDouble (calibrate_vrot,0.0))
                    calibrate_vrot = 1;
                calFile <<"  -  " <<calibrate_vrot;
            }
        }
        calFile.close();
        ROS_INFO("%i Calibration entries written to YAML", num);
    }

}


bool CalibrationMain::sendHomePositionGoal()
{
    MoveBaseClient ac_move("move_base", true);

    //wait for the action server to come up
    while(!ac_move.waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for the move_base action server to come up");
    }
    if(ac_move.isServerConnected())
    {
        move_base_msgs::MoveBaseGoal goal;

        //goal.target_pose.header.frame_id = "base_link";
        goal.target_pose.header.frame_id = "/map";
        goal.target_pose.header.stamp = ros::Time::now();

        goal.target_pose.pose = homeTransform;

        ROS_INFO("Sending robot home");
        ac_move.sendGoal(goal);

        ac_move.waitForResult();

        if(ac_move.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
          ROS_INFO("Hooray, we reached home!");
          return true;
        }
        else
        {
          ROS_INFO("The base failed to move for some reason");
          return false;
        }
    }
    else
    {
        ROS_INFO("could not connect to move_base server");
        return false;
    }
}


bool CalibrationMain::startContinuousRun()
{
    double xSpeed = ui->SpinBoxXSpeed->value();
    double rotSpeed = ui->SpinBoxRotSpeed->value();
    double xSpeedInc = ui->SpinBoxContXSpeed->value();
    double rotSpeedInc = ui->SpinBoxContRotSpeed->value();
    double time = ui->SpinBoxContCalTime->value();
    bool safeHome = false;

    iterations = ui->SpinBoxNumIt->value();
    vxvrInd = ui->CheckVxVrSeparate->isChecked();

    ROS_INFO("Continuous run started");

    goals.clear(); // fresh start for new run
    results.clear(); // fresh start for new run

    // setting home position
    tf::StampedTransform transform;
    try
    {
        listener->lookupTransform("/map","/base_footprint",ros::Time(0.0), transform);
    }
    catch (tf::TransformException ex)
    {
         ROS_ERROR("Nope! %s", ex.what());
         return false;
    }
    tf::poseTFToMsg(transform, homeTransform);

    if(vxvrInd)
    {
        // vx and vr are independent -> need less runs
        for(int i = 0; i<iterations; i++)
        {
            double _xSpeed = xSpeed + i*xSpeedInc;
            double _rotSpeed = rotSpeed + i*rotSpeedInc;
            send_goal(_xSpeed,_rotSpeed, time);
            // wait until calibration run is finished, result stored in callback
            ac.waitForResult();

            safeHome = sendHomePositionGoal(); // function waits until we reach home or goal is aborted
            if(!safeHome || !goalSucceeded) // abort runs if we did not reach home position or run was not successful
            {
                printStoreToYAML();
                return false;
            }
        }
    }
    else
    {
        // vx and vr are dependent -> check everything separately
        // outer loop for xSpeed, inner loop for rotSpeed
        for(int i = 0; i<iterations; i++)
        {
            double _xSpeed = xSpeed + i*xSpeedInc;
            for(int j = 0; j<iterations; j++)
            {
                double _rotSpeed = rotSpeed + j*rotSpeedInc;
                send_goal(_xSpeed,_rotSpeed, time);
                // wait until calibration run is finished, result stored in callback
                ac.waitForResult();

                safeHome = sendHomePositionGoal(); // function waits until we reach home or goal is aborted
                if(!safeHome || !goalSucceeded) // abort runs if we did not reach home position or run was not successful
                {
                    printStoreToYAML();
                    return false;
                }
                if(rotSpeedInc == 0.0) // if there's no increase in rotational speed we don't need to iterate
                {
                    break;
                }
            }
            if(xSpeedInc == 0.0) // if there's no increase in translational speed we don't need to iterate
            {
                break;
            }
        }
    }

    // write YAML file here from stored results
    printStoreToYAML();
    return true;
}






