#include "autoware_rosbag_plugin.h"
#include "ui_autoware_rosbag_plugin.h"
#include <QTimer>
#include <QFileDialog>
#include <QSlider>

#include <sstream>
#include <iostream>
#include <fstream>
#include <cmath>
#include <iostream>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <signal.h>
#include <unistd.h>
#include <boost/thread.hpp>
#include <boost/shared_ptr.hpp>

#include <ros/ros.h>
#include <ros/common.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <rosgraph_msgs/Clock.h>


using namespace std;
//using namespace rosbag_control;

const QString Autoware_Rosbag_Plugin::DEFAULT_SAVE_PATH = "/home/user/";
const int     Autoware_Rosbag_Plugin::TIMER_FREQ     = 1000;

int test_count = 1;



Autoware_Rosbag_Plugin::Autoware_Rosbag_Plugin(QWidget *parent) :
    rviz::Panel(parent),
    ui(new Ui::Autoware_Rosbag_Plugin)
{
  record_filepath_.clear();
  record_filename_.clear();

  const char* DEFAULT_RECORD_TOPICS[] = {
      "/velodyne_packets",
      "/velodyne_points",
  };

  record_topics_.clear();
  for(size_t i=0; i< (sizeof(DEFAULT_RECORD_TOPICS)/sizeof(const char*)); i++)
  {
    record_topics_.push_back( DEFAULT_RECORD_TOPICS[i] );
  }


  ui->setupUi(this);
  ui_timer_ = new QTimer();
  ui_timer_->setInterval(TIMER_FREQ);
  QObject::connect(ui_timer_, SIGNAL(timeout()), this, SLOT(timeShow()));

  ui->button_record_start->setDisabled(true);
  ui->button_record_stop->setDisabled(true);


}

Autoware_Rosbag_Plugin::~Autoware_Rosbag_Plugin()
{
  delete ui;
  delete ui_timer_;
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(Autoware_Rosbag_Plugin, rviz::Panel)

void Autoware_Rosbag_Plugin::on_edit_record_filename_textChanged(const QString &arg1)
{
  if( !arg1.isEmpty() )
  {
    /* set filename, activate record start */
    record_filename_ = arg1.toStdString();
    ui->button_record_start->setEnabled(true);
  }

}

void Autoware_Rosbag_Plugin::on_button_record_save_clicked()
{

  /* open dialog */
  QString pathInfo =
      QFileDialog::getSaveFileName(
              this,
              tr("path of saving bag"),
              DEFAULT_SAVE_PATH);

  if( ! pathInfo.isEmpty() )
  {
    QString filepath, filename, strTemp;

    strTemp = pathInfo;
    int idx = strTemp.lastIndexOf('/');

    if( idx != -1 )
    {
      filepath = strTemp.left(idx+1);
      filename = strTemp.remove(0, idx+1);
    }
    else
    {
      ROS_ERROR("Invalid Path for recording!!");
      return;
    }

    idx = filename.lastIndexOf(".bag");
    if(idx == -1)
    {
      filename.append(".bag");
    }

    record_filepath_ = filepath.toStdString();
    ui->edit_record_filename->setText(filename);

  }

}

void Autoware_Rosbag_Plugin::on_button_record_start_clicked()
{
//  Rosbag_Control_Manager::RequestInfo reqInfo;
  RecordParam recoParam;
////////////////////////////////////////////////////
//  reqInfo.req_type = Rosbag_Control_Manager::ROSBAG_REQ_RECORD;
//  /* Set Split Param*/
//  if(record_duration_secs_ != 0)
//  {
//    recInfo.max_duration = record_duration_secs_;
//  }

//  reqInfo.param.recInfo = &recInfo;
//////////////////////////////////////////////////

//  /* Set Record Topics Param */
  record_topics_.clear();
  if( ui->checkBox_1->isChecked() )
    record_topics_.push_back("/velodyne_packets");
  if( ui->checkBox_2->isChecked() )
    record_topics_.push_back("/velodyne_points");
  if( ui->checkBox_3->isChecked() )
    record_topics_.push_back("/image_raw");

  std::vector<std::string>::iterator ite = record_topics_.begin();
  while( ite != record_topics_.end() )
  {
    recoParam.topics.push_back(*ite);
    ite++;
  }

  /* Set Filename Param */
  recoParam.filename = record_filepath_ + record_filename_;

  /* Start Record Request */
  recordReq(recoParam);

  /*Start Timer*/
  ui_timer_->start();


///////////////////////////////////////////
  ui->button_record_save->setDisabled(true);
  ui->button_record_stop->setEnabled(true);
  ui->button_record_start->setDisabled(true);
//  ui->grp_play->setDisabled(true);
//  ui->btn_recordStart->setDisabled(true);
//  ui->btn_saveFile->setDisabled(true);
//  ui->chkbx_split->setDisabled(true);
//  setSplitGroupState();

//  ui->chkbx_timestamp->setDisabled(true);
//  ui->btn_recordStop->setEnabled(true);
///////////////////////////////////////////


  /* Start Timer */
//  setTimerState(true);

//  ROS_INFO("%s L.%d - Enetered", __FUNCTION__, __LINE__);
//  int ret = ROSBAG_RET_NG;

}

int Autoware_Rosbag_Plugin::recordReq( RecordParam &recoParam )
{
//  ROS_INFO("%s L.%d - Enetered", __FUNCTION__, __LINE__);
  recorder_opts_.reset( new rosbag_control::RecorderOptions() );
  if (recoParam.filename.empty())
  {
    recorder_opts_->append_date = true;
  }
  recorder_opts_->prefix = recoParam.filename;
  recorder_opts_->append_date = false;

  /* Set Default Options */
  recorder_opts_->buffer_size   = 1048576 * 256;
  recorder_opts_->chunk_size    = 1024 * 768;
  recorder_opts_->min_space     = 1 * 1073741824ull;
  recorder_opts_->min_space_str = "1G";
  recorder_opts_->time_publish  = false;

  /* Set Max Duration Sec */
  if( recoParam.max_duration != 0 )
  {
    recorder_opts_->split = true;
    recorder_opts_->max_duration = ros::Duration(recoParam.max_duration);
  }

  /* Set record topics */
  if( recoParam.topics.empty() )
  {
//    ROS_INFO("%s L.%d - Set all topic record option", __FUNCTION__, __LINE__, ret);
    recorder_opts_->record_all = true;
  }
  else
  {
    /* Record Topics Setting */
    std::vector<std::string>::iterator ite = recoParam.topics.begin();
    while(ite < recoParam.topics.end() )
    {
      recorder_opts_->topics.push_back( *ite );
      ite++;
    }
  }

  ROS_INFO("%s L.%d - Set Recorder Option", __FUNCTION__, __LINE__);
  ROS_INFO("%s L.%d -   record_all      [%d]", __FUNCTION__, __LINE__, recorder_opts_->record_all);
  ROS_INFO("%s L.%d -   regex           [%d]", __FUNCTION__, __LINE__, recorder_opts_->regex);
  ROS_INFO("%s L.%d -   do_exclude      [%d]", __FUNCTION__, __LINE__, recorder_opts_->do_exclude);
  ROS_INFO("%s L.%d -   quiet           [%d]", __FUNCTION__, __LINE__, recorder_opts_->quiet);
  ROS_INFO("%s L.%d -   append_date     [%d]", __FUNCTION__, __LINE__, recorder_opts_->append_date);
  ROS_INFO("%s L.%d -   verbose         [%d]", __FUNCTION__, __LINE__, recorder_opts_->verbose);
  ROS_INFO("%s L.%d -   compression     [%d]", __FUNCTION__, __LINE__, recorder_opts_->compression);
  ROS_INFO("%s L.%d -   prefix          [%s]", __FUNCTION__, __LINE__, recorder_opts_->prefix.c_str());
  ROS_INFO("%s L.%d -   name            [%s]", __FUNCTION__, __LINE__, recorder_opts_->name.c_str());
  ROS_INFO("%s L.%d -   buffer_size     [%d]", __FUNCTION__, __LINE__, recorder_opts_->buffer_size);
  ROS_INFO("%s L.%d -   chunk_size      [%d]", __FUNCTION__, __LINE__, recorder_opts_->chunk_size);
  ROS_INFO("%s L.%d -   limit           [%d]", __FUNCTION__, __LINE__, recorder_opts_->limit);
  ROS_INFO("%s L.%d -   split           [%d]", __FUNCTION__, __LINE__, recorder_opts_->split);
  ROS_INFO("%s L.%d -   max_size        [%d]", __FUNCTION__, __LINE__, recorder_opts_->max_size);
  ROS_INFO("%s L.%d -   max_duration    [%lf]", __FUNCTION__, __LINE__, recorder_opts_->max_duration.toSec() );
  ROS_INFO("%s L.%d -   node            [%s]", __FUNCTION__, __LINE__, recorder_opts_->node.c_str() );
  ROS_INFO("%s L.%d -   min_space       [%d]", __FUNCTION__, __LINE__, recorder_opts_->min_space );
  ROS_INFO("%s L.%d -   min_space_str   [%s]", __FUNCTION__, __LINE__, recorder_opts_->min_space_str.c_str() );
  ROS_INFO("%s L.%d -   time_publish    [%d]", __FUNCTION__, __LINE__, recorder_opts_->time_publish );
  ROS_INFO("%s L.%d -   topic num       [%d]", __FUNCTION__, __LINE__, recorder_opts_->topics.size() );

  for( size_t i=0; i<recorder_opts_->topics.size(); i++ ) {
    ROS_INFO("%s L.%d - [%d] %s", __FUNCTION__, __LINE__, i, recorder_opts_->topics.at(i).c_str() );
  }


  doRecord( *recorder_opts_ );


//  ROS_INFO("%s L.%d - Returned [%d]", __FUNCTION__, __LINE__, ret);
  return 0;
}

int Autoware_Rosbag_Plugin::doRecord( rosbag_control::RecorderOptions &opt )
{
  ROS_INFO("%s L.%d - Enetered", __FUNCTION__, __LINE__);
//  int ret = ROSBAG_RET_NG;

  recorder_.reset(new rosbag_control::Recorder(opt) );
  record_status = 1;

  record_time_start_ = ros::Time::now();

  if( recorder_->start() == 0 )
  {
//    setState( ROSBAG_STATE_RECORD );
//    ret = ROSBAG_RET_OK;
//   ROS_INFO("Error???");
  }
  else
  {
    recorder_.reset();
    recorder_opts_.reset();
  }

//  ROS_INFO("%s L.%d - Return [%d] ", __FUNCTION__, __LINE__, ret);
  return 0;
}

void Autoware_Rosbag_Plugin::on_button_record_stop_clicked()
{
  ROS_INFO("%s L.%d - Enetered", __FUNCTION__, __LINE__);

  recorder_->stop();
  recorder_.reset();
  recorder_opts_.reset();

  ui->button_record_start->setEnabled(true);
  ui->button_record_save->setEnabled(true);
  ui->button_record_stop->setDisabled(true);
  record_status = 0;

//  record_current_time_ = 0;
  ros::Duration record_time_reset_ = ros::Duration(0);
  Autoware_Rosbag_Plugin::updateRecordTime(record_time_reset_);

  ROS_INFO("%s L.%d - Returned", __FUNCTION__, __LINE__);

}

void Autoware_Rosbag_Plugin::updateRecordTime(ros::Duration record_time_visual)
{

  QString fmt("hh:mm:ss");
  QTime bufTime(0,0,0);

  QTime curTime = bufTime.addSecs(record_time_visual.toSec());
  QString buf = curTime.toString(fmt);

  ui->label_recTime->setText(buf);


}

void Autoware_Rosbag_Plugin::timeShow()
{
  ros::Duration record_time_duration_;
  if (record_status == 1)
    record_time_duration_ = ros::Time::now() - record_time_start_;
  else
    record_time_duration_ = ros::Time::now() - ros::Time::now();
  updateRecordTime(record_time_duration_);
}

void Autoware_Rosbag_Plugin::on_botton_topic_refresh_clicked()
{
//  QPushButton *dynamic1= new QPushButton(ui->button_record_save);

  ros::master::V_TopicInfo master_topics;
  ros::master::getTopics(master_topics);

  for (ros::master::V_TopicInfo::iterator it = master_topics.begin() ; it != master_topics.end(); it++) {
    const ros::master::TopicInfo& info = *it;
    std::cout << "topic_" << it - master_topics.begin() << ": " << info.name << std::endl;
  }

  QLayout* layout = ui->widget_topic->layout ();
  if (layout != 0)
  {
    QLayoutItem *item;
    while ((item = layout->takeAt(0)) != 0)
    {
      delete item->widget();
      delete item;
    }
    delete layout;
  }

  QVBoxLayout *lay = new QVBoxLayout(this);
  if (test_count % 3 == 0)
  {
    QCheckBox *dynamic = new QCheckBox("This is a check box");
    dynamic->setChecked (true);
    lay->addWidget(dynamic);
  }
  else
//    for (int i = 0; i < record_topics_.size(); i++)

    for (std::vector<std::string>::iterator it = record_topics_.begin(); it != record_topics_.end(); it++)
    {
      QCheckBox *dynamic = new QCheckBox(QString::fromStdString(*it));
      dynamic->setChecked (true);
      lay->addWidget(dynamic);
    }



  ui->widget_topic->setLayout(lay);
//  ui->widget_topic->update();
  test_count++;

//  dynamic1.setText("Test1");



//  ui->gridLayout->addItem(dynamic1);

//  QVBoxLayout layout;
//  layout.addWidget(&dynamic1);


}
