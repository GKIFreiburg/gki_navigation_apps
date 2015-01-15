#include "calibrationmain.h"
#include "ui_calibrationmain.h"

#include "boost/date_time/posix_time/posix_time.hpp"


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
        resultFile << "This is the result file (created on " <<date <<")\n\n";
        resultFile.close();
    }
    f.close();
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
    //count++;
    //QString value = QString::number(count);
    //ui->labelStatus->setText("clicked" + value);
    double xSpeed = ui->SpinBoxXSpeed->value();
    double rotSpeed = ui->SpinBoxRotSpeed->value();
    double time = ui->SpinBoxRotSpeed_2->value();

    send_goal(xSpeed, rotSpeed, time);
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

    if(state == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      ROS_INFO("state is succeeded");

      calibrate_twist::CalibrateResultConstPtr result = ac.getResult();

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
    string date = to_simple_string(ros::Time::now().toBoost());
    //ofstream resultFile(filename.c_str(), ios::out | ios::app);
    ofstream resultFile("result.txt", ios::out | ios::app);
    resultFile << "Calibration run on " <<date;
    resultFile << "\n\tGoal: vx: " << currentGoal.twist_goal.linear.x <<" vrot: " <<currentGoal.twist_goal.angular.z // goal
               << "\n\tOdoResult: vx: " <<result->odo_result.twist.linear.x <<" vrot: " <<result->odo_result.twist.angular.z // result
               <<"\n\tResult: vx: " <<result->calibrated_result.twist.linear.x <<" vrot: " <<result->calibrated_result.twist.angular.z // result
               <<"\n\tCalibration value: vx: " <<calibrate_vx <<" vrot: " <<calibrate_vrot <<"\n\n"; //calibration value
    resultFile.close();
    ROS_INFO("Result written to file");
}










