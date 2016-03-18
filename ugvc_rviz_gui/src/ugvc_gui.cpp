#include "ugvc_gui.h"
#include "ui_ugvc_gui.h"
#include <iostream>
#include "rviz/view_manager.h"
#include "rviz/tool_manager.h"
#include "rviz/properties/property_tree_model.h"

/**
 * This class creates the GUI using rviz APIs.
 */

UgvcGUI::UgvcGUI(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::UgvcGUI),it_(nh_)
{
    /**
     * Set up the QT related UI components.
     */
    ui->setupUi(this);
    ui->sliderLinearVel->setValue(75);
    ui->sliderAngularVel->setValue(75);
    ui->lbLightingItem1->setStyleSheet("QLabel { background-color : red; color : rgb(255, 255, 255); }");
    ui->lbLightingItem2->setStyleSheet("QLabel { background-color : green; color : rgb(255, 255, 255); }");
    ui->lbFloorsItem1->setStyleSheet("QLabel { background-color : green; color : rgb(255, 255, 255); }");
    ui->lbFloorsItem2->setStyleSheet("QLabel { background-color : green; color : rgb(255, 255, 255); }");
    ui->lbStairsItem1->setStyleSheet("QLabdel { background-color : green; color : rgb(255, 255, 255); }");
    ui->lbStairsItem2->setStyleSheet("QLabel { background-color : green; color : rgb(255, 255, 255); }");
    ui->lbStairsItem3->setStyleSheet("QLabel { background-color : red; color : rgb(255, 255, 255); }");
    ui->lbBedroomItem1->setStyleSheet("QLabel { background-color : green; color : rgb(255, 255, 255); }");
    ui->lbBedroomItem2->setStyleSheet("QLabel { background-color : red; color : rgb(255, 255, 255); }");
    ui->lbBedroomItem3->setStyleSheet("QLabel { background-color : green; color : rgb(255, 255, 255); }");
    ui->lbPetItem1->setStyleSheet("QLabel { background-color : yellow; color : rgb(255, 255, 255); }");
    changeToolBtnStatus(-2); //set the initial rviz tool to be "interact"

    initVariables();
    initDisplayWidgets();
    initTools();
    initActionsConnections();
}

UgvcGUI::~UgvcGUI()
{
    delete ui;
    delete mapManager_;
    delete mapRenderPanel_;
    delete manager_;
    delete renderPanel_;
    delete status_label_;
}

void UgvcGUI::initVariables()
{
    /**
     *Initialize default values of all the variables. Push these definitions to xml/config file in future
     */
    fixedFrame_ =  QString("/map");
    targetFrame_ =  QString("/camera_rgb_optical_frame");
    mapTopic_ = QString("/map");
    imageTopic_ = QString("/usb_cam/image_raw");
    pointCloudTopic_=QString("/camera/depth/points");
    octomapTopic_=QString( "/occupied_cells_vis_array" );
    baseSensorTopic_=QString("/mobile_base/sensors/core");
    velocityTopic_=QString("/turtle1/cmd_vel");
    pathTopic_ = QString("/move_base/NavfnROS/plan");

    moveBaseCmdPub = nh_.advertise<geometry_msgs::Twist>(velocityTopic_.toStdString(),1);
    centerDistSub = nh_.subscribe("/distance/image_center_dist",1,&UgvcGUI::distanceSubCallback,this);
    //baseSensorStatus = nh_.subscribe(baseSensorTopic_.toStdString(),1,&UgvcGUI::baseStatusCheck,this);
    liveVideoSub = it_.subscribe(imageTopic_.toStdString(),1,&UgvcGUI::liveVideoCallback,this,image_transport::TransportHints("compressed"));

    setRobotVelocity();
}

void UgvcGUI::initActionsConnections()
{
    /**
     * Set up the status Bar and display messages emitted from each of the tools.
     * All the tools in rviz API has updateStatus function to emit messages to the status bar.
     */
    status_label_ = new QLabel("");
    statusBar()->addPermanentWidget( status_label_,1);
    connect( manager_, SIGNAL( statusUpdate( const QString& )), status_label_, SLOT( setText( const QString& )));

    /**
     * Setup Signals and slots for different buttons/sliders in UI.
     */
    connect(ui->btnUp, SIGNAL(clicked()), this, SLOT(moveBaseForward()));
    connect(ui->btnDown, SIGNAL(clicked()), this, SLOT(moveBaseBackward()));
    connect(ui->btnLeft, SIGNAL(clicked()), this, SLOT(moveBaseLeft()));
    connect(ui->btnRight, SIGNAL(clicked()), this, SLOT(moveBaseRight()));

    connect(ui->btnGroupRvizTools,SIGNAL(buttonClicked(int)),this,SLOT(setCurrentTool(int)));

    connect(ui->sliderLinearVel, SIGNAL(valueChanged(int)),this,SLOT(setRobotVelocity()));
    connect(ui->sliderAngularVel, SIGNAL(valueChanged(int)),this,SLOT(setRobotVelocity()));

    connect(ui->tab_display, SIGNAL(currentChanged(int)),this,SLOT(setActiveRvizToolBtns(int)));
}

void UgvcGUI::initDisplayWidgets()
{

    //Setup the UI elements for displaying 2D map
    /**
     * VisualizationManager is used to control different displays that are shown in a widget.
     * Renderpanel is a widget that provides a 3D space in the visualizationmanager.
     * startUpdate() function starts the timers and subscribes to defined topics at 30Hz.
     */
    mapRenderPanel_ = new rviz::RenderPanel();
    ui->map_layout->addWidget(mapRenderPanel_);
    mapManager_ = new rviz::VisualizationManager( mapRenderPanel_ );
    mapRenderPanel_->initialize( mapManager_->getSceneManager(), mapManager_);
    mapManager_->setFixedFrame(fixedFrame_);
    mapManager_->initialize();
    mapManager_->startUpdate();

    //Create and assign FixedOrientationOrthoViewController to the existing viewmanager of the visualization manager
    /**
     * VisualisationManager has a manager for most of its children. ViewManager is responsible for setting the viewController.
     * Default View Controller is rviz/Orbit, for map we are changing it to rviz/TopDownOrtho
     * To set properties of most of the rviz objects, use subProp and setValue functions as shown below
     * New displays can be created and added to the visualization manager using createDisplay function as used below
     *
     * @todo Create an xml/config file to define objects to be displayed in GUI alongwith their parameters
     */
    mapViewManager_ = mapManager_->getViewManager();
    mapViewManager_->setCurrentViewControllerType("rviz/TopDownOrtho");
    mapViewController_ = mapViewManager_->getCurrent();

    //Set parameters of the view controller to show map correctly
    mapViewController_->subProp("X")->setValue(0);
    mapViewController_->subProp("Y")->setValue(0);
    mapViewController_->subProp("Angle")->setValue(0);
    mapViewController_->subProp("Scale")->setValue(20);

    // Create a map display
    mapDisplay_ = mapManager_->createDisplay( "rviz/Map", "2D Map view", true );
    ROS_ASSERT( mapDisplay_ != NULL );

    mapDisplay_->subProp( "Topic" )->setValue( mapTopic_ );

    mapManager_->createDisplay( "rviz/RobotModel", "Turtlebot", true );

    mapManager_->createDisplay("rviz/Path","Global path",true)->subProp( "Topic" )->setValue(pathTopic_);


    // Initialize GUI elements for main panel
    renderPanel_ = new rviz::RenderPanel();
    ui->display3d_layout->addWidget(renderPanel_);

    manager_ = new rviz::VisualizationManager( renderPanel_ );
    renderPanel_->initialize( manager_->getSceneManager(), manager_ );

    //set the fixed frame before initializing Visualization Manager. pointcloud2 will not work with this
    manager_->setFixedFrame(fixedFrame_);
    manager_->initialize();
    manager_->startUpdate();


    // Create a main display to show pointcloud and octomap

    manager_->createDisplay( "rviz/Grid", "Grid", true );
    manager_->createDisplay( "rviz/RobotModel", "Turtlebot", true );

    octomapDisplay_ = manager_->createDisplay( "rviz/MarkerArray", "Octomap view", true );
    ROS_ASSERT( octomapDisplay_ != NULL );

    octomapDisplay_->subProp( "Marker Topic" )->setValue(octomapTopic_);

    //Assign Target Frame to the existing viewmanager of the visualization manager
    rviz::ViewManager* viewManager_ = manager_->getViewManager();
    rviz::ViewController* viewController_ = viewManager_->getCurrent();
    viewController_->subProp("Target Frame")->setValue(targetFrame_);
    manager_->createDisplay("rviz/Path","Global path",true)->subProp( "Topic" )->setValue(pathTopic_);

}

void UgvcGUI::initTools(){
    /**
     * ToolManager is similar to ViewManager. It can be used to add new tools and change the current or default tool.
     * Properties of tools are stored in a PropertyTreeModel. To set/modify any property of a tool use getPropertyContainer function.
     */
    toolManager_ = manager_->getToolManager();

    pointTool_ = toolManager_->addTool("rviz/PublishPoint");
    measureTool_ = toolManager_->addTool("rviz/Measure");
    setGoalTool_ = toolManager_->addTool("rviz/SetGoal");
    setInitialPoseTool_=toolManager_->addTool("rviz/SetInitialPose");
    interactTool_ = toolManager_->addTool("rviz/Interact");

    mapToolManager_ = mapManager_->getToolManager();

    mapInteractTool_ = mapToolManager_->addTool("rviz/Interact");
    setMapGoalTool_ = mapToolManager_->addTool("rviz/SetGoal");
    setMapInitialPoseTool_ = mapToolManager_->addTool("rviz/SetInitialPose");

    // Find the entry in propertytreemodel and set the value for Topic
    setGoalTool_->getPropertyContainer()->subProp("Topic")->setValue("/move_base_simple/goal");
    setMapGoalTool_->getPropertyContainer()->subProp("Topic")->setValue("/move_base_simple/goal");

}

void UgvcGUI::keyPressEvent(QKeyEvent *event)
{
    switch(event->key())
    {
    case Qt::Key_W:
        moveBaseForward();
        ROS_INFO("key W pressed");
        break;
    case Qt::Key_A:
        moveBaseLeft();
        ROS_INFO("key A pressed");
        break;
    case Qt::Key_D:
        moveBaseRight();
        ROS_INFO("key D pressed");
        break;
    case Qt::Key_S:
        moveBaseBackward();
        ROS_INFO("key S pressed");
        break;
    default:
        QWidget::keyPressEvent(event);
        break;
    }
}

void UgvcGUI::distanceSubCallback(const std_msgs::Float32::ConstPtr& msg)
{
    QLocale english(QLocale::English, QLocale::UnitedStates);
    QString qdist = english.toString(msg->data, 'f', 2);
    ui->lbDistance->setText(qdist);
}


void UgvcGUI::liveVideoCallback(const sensor_msgs::ImageConstPtr& msg)
{

    /**
     * Adding Image display opens up the image in a new window.
     * As a workaround to show image in the same GUI window, OpenCV is being used to display image on a Qlabel
     */
    cv_bridge::CvImagePtr cv_ptr, cv_ptr_big;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        cv_ptr_big = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    //  convert cv image into RGB image and resize it to the size of available layout
    setVideo(ui->liveVideoLabel,cv_ptr);
    setVideo(ui->lbLiveVideoBig,cv_ptr_big);
}

void UgvcGUI::setVideo(QLabel* label, cv_bridge::CvImagePtr cv_ptr){
    cv::Mat RGBImg;
    QLabel* liveVideoLabel = label;

    // To avoid auto expansion of QLabel,keep the video dimensions slightly less than the label dimension
    int height = liveVideoLabel->height()-1;
    int width =  liveVideoLabel->width()-1;

    if(liveVideoLabel->height()-1 >= (liveVideoLabel->width()-1)*3/4)
        height= (liveVideoLabel->width()-1)*3/4;
    else
        width = (liveVideoLabel->height()-1)*4/3;

    cv::cvtColor(cv_ptr->image,RGBImg,CV_BGR2RGB);
    cv::resize(RGBImg,RGBImg,cvSize(width,height));

    //  convert RGB image into QImage and publish that on the label for livevideo
    QImage qImage_= QImage((uchar*) RGBImg.data, RGBImg.cols, RGBImg.rows, RGBImg.cols*3, QImage::Format_RGB888);
    liveVideoLabel->setPixmap(QPixmap::fromImage(qImage_));
    liveVideoLabel->show();

}

void UgvcGUI::setRobotVelocity()
{
    linearVelocity = ui->sliderLinearVel->value()*(LIN_VEL_MAX-LIN_VEL_MIN)/100+LIN_VEL_MIN;
    ROS_INFO("Linear velocity:%f",linearVelocity);
    angularVelocity = ui->sliderAngularVel->value()*(ANG_VEL_MAX-ANG_VEL_MIN)/100+ANG_VEL_MIN;
    ROS_INFO("Angular velocity:%f",angularVelocity);
}

void UgvcGUI::moveBaseForward()
{
    ROS_INFO("move forward");

    moveBaseCmd.linear.x=linearVelocity;
    moveBaseCmd.linear.y=0;
    moveBaseCmd.linear.z=0;

    moveBaseCmd.angular.x=0;
    moveBaseCmd.angular.y=0;
    moveBaseCmd.angular.z=0;

    sendMoveBaseCmd();
}

void UgvcGUI::moveBaseBackward()
{
    ROS_INFO("move backward");

    moveBaseCmd.linear.x=-linearVelocity;
    moveBaseCmd.linear.y=0;
    moveBaseCmd.linear.z=0;

    moveBaseCmd.angular.x=0;
    moveBaseCmd.angular.y=0;
    moveBaseCmd.angular.z=0;

    sendMoveBaseCmd();
}

void UgvcGUI::moveBaseLeft()
{
    ROS_INFO("move left");

    moveBaseCmd.linear.x=0;
    moveBaseCmd.linear.y=0;
    moveBaseCmd.linear.z=0;

    moveBaseCmd.angular.x=0;
    moveBaseCmd.angular.y=0;
    moveBaseCmd.angular.z=angularVelocity;

    sendMoveBaseCmd();
}

void UgvcGUI::moveBaseRight()
{
    ROS_INFO("move right");

    moveBaseCmd.linear.x=0;
    moveBaseCmd.linear.y=0;
    moveBaseCmd.linear.z=0;

    moveBaseCmd.angular.x=0;
    moveBaseCmd.angular.y=0;
    moveBaseCmd.angular.z=-angularVelocity;

    sendMoveBaseCmd();
}

void UgvcGUI::sendMoveBaseCmd()
{
    if(ros::ok() && moveBaseCmdPub)
    {
        moveBaseCmdPub.publish(moveBaseCmd);
        ROS_INFO("move base cmd sent");
    }
}

void UgvcGUI::setCurrentTool(int btnID)
{
    if(btnID == -2)
    {
        ROS_INFO("Interact Tool Selected");
        toolManager_->setCurrentTool(interactTool_);        
        mapToolManager_->setCurrentTool(mapInteractTool_);

    }
    else if(btnID == -3)
    {
        ROS_INFO("Measure Tool Selected");
        toolManager_->setCurrentTool(measureTool_);

    }
    else if(btnID == -4)
    {
        ROS_INFO("2DPoseEstimate Tool Selected");
        toolManager_->setCurrentTool(setInitialPoseTool_);
        mapToolManager_->setCurrentTool(setMapInitialPoseTool_);
    }
    else if(btnID == -5)
    {
        ROS_INFO("2DNavGoal Tool Selected");
        toolManager_->setCurrentTool(setGoalTool_);
        mapManager_->getToolManager()->setCurrentTool(setMapGoalTool_);
    }
    else if(btnID == -6)
    {
        ROS_INFO("PublishPoint Tool Selected");
        toolManager_->setCurrentTool(pointTool_);
    }

    changeToolBtnStatus(btnID);
}

void UgvcGUI::changeToolBtnStatus(int btnID)
{
    ui->btnRvizInteract->setFlat(true);
    ui->btnRvizMeasure->setFlat(true);
    ui->btnRvizNavGoal->setFlat(true);
    ui->btnRvizPoseEstimate->setFlat(true);
    ui->btnRvizPublishPoint->setFlat(true);

    switch(btnID)
    {
    case -2: ui->btnRvizInteract->setFlat(false);
        break;
    case -3: ui->btnRvizMeasure->setFlat(false);
        break;
    case -4: ui->btnRvizPoseEstimate->setFlat(false);
        break;
    case -5: ui->btnRvizNavGoal->setFlat(false);
        break;
    case -6: ui->btnRvizPublishPoint->setFlat(false);
    }
}

void UgvcGUI::setActiveRvizToolBtns(int tabID)
{
//    ROS_INFO("TAB:%d",tabID);

    ui->btnRvizInteract->setDisabled(false);
    ui->btnRvizMeasure->setDisabled(false);
    ui->btnRvizPoseEstimate->setDisabled(false);
    ui->btnRvizNavGoal->setDisabled(false);
    ui->btnRvizPublishPoint->setDisabled(false);

    if(tabID == 1)
    {
        ui->btnRvizMeasure->setDisabled(true);
        ui->btnRvizPublishPoint->setDisabled(true);
    }
    else if(tabID == 2)
    {
        ui->btnRvizInteract->setDisabled(true);
        ui->btnRvizMeasure->setDisabled(true);
        ui->btnRvizPoseEstimate->setDisabled(true);
        ui->btnRvizNavGoal->setDisabled(true);
        ui->btnRvizPublishPoint->setDisabled(true);
    }
}



/*
//Image :
    grid_ = manager_->createDisplay( "rviz/Image", "Image View", true );
    ROS_ASSERT( grid_ != NULL );
    grid_->subProp( "Image Topic" )->setValue( "/camera/rgb/image_raw" );
    grid_->subProp( "Transport Hint" )->setValue( "theora" );


//Depth Cloud :
    grid_ = manager_->createDisplay( "rviz/DepthCloud", "Image View", true );
    ROS_ASSERT( grid_ != NULL );

    grid_->subProp( "Depth Map Topic" )->setValue( "/camera/depth/image_raw" );
    grid_->subProp( "Depth Map Transport Hint" )->setValue( "raw" );
    grid_->subProp( "Color Image Topic" )->setValue( "/camera/rgb/image_raw" );
    grid_->subProp( "Color Transport Hint" )->setValue( "raw" );
    grid_->subProp("Queue Size")->setValue(5);
    grid_->subProp("Style")->setValue("Flat Squares");

//    mainDisplay_ = manager_->createDisplay( "rviz/PointCloud2", "3D Pointcloud view", true );
//    ROS_ASSERT( mainDisplay_ != NULL );

//    mainDisplay_->subProp( "Topic" )->setValue( pointCloudTopic_ );
//    mainDisplay_->subProp( "Selectable" )->setValue( "true" );
//    mainDisplay_->subProp( "Style" )->setValue( "Boxes" );
//    mainDisplay_->subProp("Alpha")->setValue(0.5);

  manager_->createDisplay( "rviz/Grid", "Grid", true );

//MarkerArray :
rviz::Display* octomapDisplay_ = manager_->createDisplay( "rviz/MarkerArray", "Octomap view", true );
ROS_ASSERT( octomapDisplay_ != NULL );

octomapDisplay_->subProp( "Marker Topic" )->setValue( "/occupied_cells_vis_array" );


*/
