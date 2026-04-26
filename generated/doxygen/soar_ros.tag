<?xml version='1.0' encoding='UTF-8' standalone='yes' ?>
<tagfile doxygen_version="1.12.0" doxygen_gitid="c73f5d30f9e8b1df5ba15a1d064ff2067cbb8267">
  <compound kind="file">
    <name>SafeQueue.hpp</name>
    <path>include/soar_ros/</path>
    <filename>SafeQueue_8hpp.html</filename>
    <class kind="class">soar_ros::SafeQueue</class>
  </compound>
  <compound kind="class">
    <name>soar_ros::ActionClient</name>
    <filename>classsoar__ros_1_1ActionClient.html</filename>
    <templarg>typename ActionT</templarg>
    <templarg>typename pGoalMsg</templarg>
    <templarg>typename pFeedbackMsg</templarg>
    <templarg>typename wrappedResultMsg</templarg>
    <base>soar_ros::Output&lt; typename ActionT::Goal::SharedPtr &gt;</base>
    <base>soar_ros::Input&lt; bool &gt;</base>
    <base>soar_ros::Input&lt; typename ActionT::Feedback::SharedPtr &gt;</base>
    <base>soar_ros::Input&lt; T &gt;</base>
    <base>soar_ros::Interface</base>
    <member kind="function">
      <type>void</type>
      <name>send_goal_from_soar</name>
      <anchorfile>classsoar__ros_1_1ActionClient.html</anchorfile>
      <anchor>a7bce21809ddf9399002a60aaf86c268c</anchor>
      <arglist>(const pGoalMsg &amp;soar_goal)</arglist>
    </member>
    <member kind="function">
      <type>std::string</type>
      <name>getTopic</name>
      <anchorfile>classsoar__ros_1_1ActionClient.html</anchorfile>
      <anchor>aa76e3e400fa7e9c2012bd3203f20a4d8</anchor>
      <arglist>() override</arglist>
    </member>
    <member kind="function">
      <type>sml::Agent *</type>
      <name>getAgent</name>
      <anchorfile>classsoar__ros_1_1ActionClient.html</anchorfile>
      <anchor>a69f666414032ca53b387c8483598aeba</anchor>
      <arglist>() override</arglist>
    </member>
    <member kind="function" protection="protected" virtualness="pure">
      <type>virtual pGoalMsg</type>
      <name>parse</name>
      <anchorfile>classsoar__ros_1_1ActionClient.html</anchorfile>
      <anchor>a48b46f50fcbc1e60037820e0d16757bf</anchor>
      <arglist>(sml::Identifier *id)=0</arglist>
    </member>
    <member kind="function" protection="protected">
      <type>void</type>
      <name>parse</name>
      <anchorfile>classsoar__ros_1_1ActionClient.html</anchorfile>
      <anchor>a1b509261bfd724290f814a0300bc5f9d</anchor>
      <arglist>(bool msg) override</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>soar_ros::Client</name>
    <filename>classsoar__ros_1_1Client.html</filename>
    <templarg>typename T</templarg>
    <templarg>typename pRequestType</templarg>
    <templarg>typename pResponseType</templarg>
    <base virtualness="virtual">soar_ros::Output&lt; typename T::Request::SharedPtr &gt;</base>
    <base virtualness="virtual">soar_ros::Input&lt; typename T::Response::SharedPtr &gt;</base>
    <base>soar_ros::Interface</base>
    <member kind="function" virtualness="pure">
      <type>pRequestType</type>
      <name>parse</name>
      <anchorfile>classsoar__ros_1_1Client.html</anchorfile>
      <anchor>abec0605464dc2a8877deacb086319df3</anchor>
      <arglist>(sml::Identifier *id) override=0</arglist>
    </member>
    <member kind="function">
      <type>std::string</type>
      <name>getTopic</name>
      <anchorfile>classsoar__ros_1_1Client.html</anchorfile>
      <anchor>ad4d5c27e5d426c0e88211c27aca30134</anchor>
      <arglist>() override</arglist>
    </member>
    <member kind="function">
      <type>sml::Agent *</type>
      <name>getAgent</name>
      <anchorfile>classsoar__ros_1_1Client.html</anchorfile>
      <anchor>a851c9c9d954740cd3384016efbd1844e</anchor>
      <arglist>() override</arglist>
    </member>
    <member kind="function" protection="protected">
      <type>void</type>
      <name>run</name>
      <anchorfile>classsoar__ros_1_1Client.html</anchorfile>
      <anchor>ac84a77a574d1156c6ad4741c43f4279c</anchor>
      <arglist>()</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>soar_ros::Input</name>
    <filename>classsoar__ros_1_1Input.html</filename>
    <templarg>typename T</templarg>
    <base>soar_ros::InputBase</base>
    <member kind="function" virtualness="pure">
      <type>virtual void</type>
      <name>parse</name>
      <anchorfile>classsoar__ros_1_1Input.html</anchorfile>
      <anchor>a09a0e5766d8c852f3da30c6bb7723ecf</anchor>
      <arglist>(T msg)=0</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>process_r2s</name>
      <anchorfile>classsoar__ros_1_1Input.html</anchorfile>
      <anchor>a24f54e499689f3751bc5ba2c37096a93</anchor>
      <arglist>() override</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>soar_ros::InputBase</name>
    <filename>classsoar__ros_1_1InputBase.html</filename>
    <member kind="function" virtualness="pure">
      <type>virtual void</type>
      <name>process_r2s</name>
      <anchorfile>classsoar__ros_1_1InputBase.html</anchorfile>
      <anchor>a54be74c01b756a1ebc1f927b5efb0f48</anchor>
      <arglist>()=0</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>soar_ros::Interface</name>
    <filename>classsoar__ros_1_1Interface.html</filename>
    <member kind="function" virtualness="pure">
      <type>virtual std::string</type>
      <name>getTopic</name>
      <anchorfile>classsoar__ros_1_1Interface.html</anchorfile>
      <anchor>a294fefd3992c72bf1c227928716e938b</anchor>
      <arglist>()=0</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual sml::Agent *</type>
      <name>getAgent</name>
      <anchorfile>classsoar__ros_1_1Interface.html</anchorfile>
      <anchor>a8e52e3b61ef77a59afa7178b681819a8</anchor>
      <arglist>()=0</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>soar_ros::Output</name>
    <filename>classsoar__ros_1_1Output.html</filename>
    <templarg>typename T</templarg>
    <base>soar_ros::OutputBase</base>
    <member kind="function">
      <type>void</type>
      <name>process_s2r</name>
      <anchorfile>classsoar__ros_1_1Output.html</anchorfile>
      <anchor>ac38d1e23a7093e233209f8ff8f8c9046</anchor>
      <arglist>(sml::Identifier *id) override</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual T</type>
      <name>parse</name>
      <anchorfile>classsoar__ros_1_1Output.html</anchorfile>
      <anchor>af01db0104f6bd68b61ac9baff164926f</anchor>
      <arglist>(sml::Identifier *id)=0</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>soar_ros::OutputBase</name>
    <filename>classsoar__ros_1_1OutputBase.html</filename>
    <member kind="function" virtualness="pure">
      <type>virtual void</type>
      <name>process_s2r</name>
      <anchorfile>classsoar__ros_1_1OutputBase.html</anchorfile>
      <anchor>a30c11674e784b63ef1cea8f3868e9e3d</anchor>
      <arglist>(sml::Identifier *id)=0</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>soar_ros::Publisher</name>
    <filename>classsoar__ros_1_1Publisher.html</filename>
    <templarg>typename T</templarg>
    <base>soar_ros::Output&lt; T &gt;</base>
    <base>soar_ros::Interface</base>
    <member kind="function" virtualness="pure">
      <type>T</type>
      <name>parse</name>
      <anchorfile>classsoar__ros_1_1Publisher.html</anchorfile>
      <anchor>a5d0151138cdd51eca9c5238f2751e66f</anchor>
      <arglist>(sml::Identifier *id) override=0</arglist>
    </member>
    <member kind="function">
      <type>std::string</type>
      <name>getTopic</name>
      <anchorfile>classsoar__ros_1_1Publisher.html</anchorfile>
      <anchor>adbcb963a82d60f1821fe78289e70481b</anchor>
      <arglist>() override</arglist>
    </member>
    <member kind="function">
      <type>sml::Agent *</type>
      <name>getAgent</name>
      <anchorfile>classsoar__ros_1_1Publisher.html</anchorfile>
      <anchor>ad2448cafbb5ebf23b9c7b528af958a3f</anchor>
      <arglist>() override</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>soar_ros::SafeQueue</name>
    <filename>classsoar__ros_1_1SafeQueue.html</filename>
    <templarg>typename T</templarg>
  </compound>
  <compound kind="class">
    <name>soar_ros::Service</name>
    <filename>classsoar__ros_1_1Service.html</filename>
    <templarg>typename T</templarg>
    <templarg>typename pRequestType</templarg>
    <templarg>typename pResponseType</templarg>
    <base virtualness="virtual">soar_ros::Input&lt; typename T::Request::SharedPtr &gt;</base>
    <base virtualness="virtual">soar_ros::Output&lt; typename T::Response::SharedPtr &gt;</base>
    <base>soar_ros::Interface</base>
    <member kind="function" virtualness="pure">
      <type>pResponseType</type>
      <name>parse</name>
      <anchorfile>classsoar__ros_1_1Service.html</anchorfile>
      <anchor>ac961b67915ab6ba21ce618dc41809e54</anchor>
      <arglist>(sml::Identifier *id) override=0</arglist>
    </member>
    <member kind="function">
      <type>std::string</type>
      <name>getTopic</name>
      <anchorfile>classsoar__ros_1_1Service.html</anchorfile>
      <anchor>a2f7c2edd8230359d73d63c720b6d8c1f</anchor>
      <arglist>() override</arglist>
    </member>
    <member kind="function">
      <type>sml::Agent *</type>
      <name>getAgent</name>
      <anchorfile>classsoar__ros_1_1Service.html</anchorfile>
      <anchor>a01b248776a0e28e387308f0796ea50f7</anchor>
      <arglist>() override</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>soar_ros::SoarAgent</name>
    <filename>classsoar__ros_1_1SoarAgent.html</filename>
    <member kind="function">
      <type></type>
      <name>SoarAgent</name>
      <anchorfile>classsoar__ros_1_1SoarAgent.html</anchorfile>
      <anchor>afd1eb552ee082e4ab71e0c1d7701abb5</anchor>
      <arglist>(sml::Agent *pAgent, rclcpp::Node::SharedPtr node)</arglist>
    </member>
    <member kind="function">
      <type>sml::Agent *</type>
      <name>getSmlAgent</name>
      <anchorfile>classsoar__ros_1_1SoarAgent.html</anchorfile>
      <anchor>a677fecc1ae2c7cdd3f1444f170a0d4dc</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>rclcpp::Node::SharedPtr</type>
      <name>getNode</name>
      <anchorfile>classsoar__ros_1_1SoarAgent.html</anchorfile>
      <anchor>a7b581eba6838b7bfeccea204f08d544b</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>addPublisher</name>
      <anchorfile>classsoar__ros_1_1SoarAgent.html</anchorfile>
      <anchor>adfbd2784f6e5b1c817b603f0bc24f663</anchor>
      <arglist>(std::shared_ptr&lt; Publisher&lt; T &gt; &gt; output)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>addPublisher</name>
      <anchorfile>classsoar__ros_1_1SoarAgent.html</anchorfile>
      <anchor>aa40853cf6e642556c2d298ca78bc2fde</anchor>
      <arglist>(std::shared_ptr&lt; Publisher&lt; T &gt; &gt; output, const std::string &amp;commandName)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>addSubscriber</name>
      <anchorfile>classsoar__ros_1_1SoarAgent.html</anchorfile>
      <anchor>a879ba8f6b717de0d598c21e3c840910e</anchor>
      <arglist>(std::shared_ptr&lt; Subscriber&lt; T &gt; &gt; input)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>addService</name>
      <anchorfile>classsoar__ros_1_1SoarAgent.html</anchorfile>
      <anchor>a5b3bcd17342c6d1454538371508cf2ab</anchor>
      <arglist>(std::shared_ptr&lt; Service&lt; T &gt; &gt; service)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>addService</name>
      <anchorfile>classsoar__ros_1_1SoarAgent.html</anchorfile>
      <anchor>adb2540175de2ebc689f523adc3897303</anchor>
      <arglist>(std::shared_ptr&lt; Service&lt; T &gt; &gt; service, const std::string &amp;commandName)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>addClient</name>
      <anchorfile>classsoar__ros_1_1SoarAgent.html</anchorfile>
      <anchor>a411f36b8ac196a6f761220b4826636ab</anchor>
      <arglist>(std::shared_ptr&lt; Client&lt; T &gt; &gt; client)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>addClient</name>
      <anchorfile>classsoar__ros_1_1SoarAgent.html</anchorfile>
      <anchor>a18dc702163715326eafe437bcef514d1</anchor>
      <arglist>(std::shared_ptr&lt; Client&lt; T &gt; &gt; client, const std::string &amp;commandName)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>addActionClient</name>
      <anchorfile>classsoar__ros_1_1SoarAgent.html</anchorfile>
      <anchor>a3c2d33a34e63890cea7f35bc2af11be0</anchor>
      <arglist>(std::shared_ptr&lt; ActionClient&lt; T &gt; &gt; action_client)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>addActionClient</name>
      <anchorfile>classsoar__ros_1_1SoarAgent.html</anchorfile>
      <anchor>aa49ac12f509e2cd275eb0e721edf7e39</anchor>
      <arglist>(std::shared_ptr&lt; ActionClient&lt; T &gt; &gt; action_client, const std::string &amp;commandName)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>updateWorld</name>
      <anchorfile>classsoar__ros_1_1SoarAgent.html</anchorfile>
      <anchor>a310416dc5e0f18e41e376bbe3189058b</anchor>
      <arglist>()</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>soar_ros::SoarRunner</name>
    <filename>classsoar__ros_1_1SoarRunner.html</filename>
    <member kind="function">
      <type>std::shared_ptr&lt; SoarAgent &gt;</type>
      <name>addAgent</name>
      <anchorfile>classsoar__ros_1_1SoarRunner.html</anchorfile>
      <anchor>abc859374ab38dd9cf0245cedb94f1fed</anchor>
      <arglist>(const std::string &amp;agent_name, const std::string &amp;source_file)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>run</name>
      <anchorfile>classsoar__ros_1_1SoarRunner.html</anchorfile>
      <anchor>adde11fa7c0683e1d29c747f5ffe09866</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>startThread</name>
      <anchorfile>classsoar__ros_1_1SoarRunner.html</anchorfile>
      <anchor>a5d3e6cdb7b55a1f8fba1f22328bd45d4</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>stopThread</name>
      <anchorfile>classsoar__ros_1_1SoarRunner.html</anchorfile>
      <anchor>a3344394558c39786279952ad381268bb</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>updateWorld</name>
      <anchorfile>classsoar__ros_1_1SoarRunner.html</anchorfile>
      <anchor>a631775299b9c20f86840e93c05620987</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>debuggerLaunch</name>
      <anchorfile>classsoar__ros_1_1SoarRunner.html</anchorfile>
      <anchor>a5a85184219cdedb2e9826481e4be5f49</anchor>
      <arglist>(const std::shared_ptr&lt; rmw_request_id_t &gt; request_header, std::shared_ptr&lt; std_srvs::srv::Trigger::Request &gt; request, std::shared_ptr&lt; std_srvs::srv::Trigger::Response &gt; response)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>getSoarKernelStatus</name>
      <anchorfile>classsoar__ros_1_1SoarRunner.html</anchorfile>
      <anchor>af7c28fc3d7ba9f4d9b2178fca86c230c</anchor>
      <arglist>(const std::shared_ptr&lt; rmw_request_id_t &gt; request_header, std::shared_ptr&lt; std_srvs::srv::Trigger::Request &gt; request, std::shared_ptr&lt; std_srvs::srv::Trigger::Response &gt; response)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>runSoarKernel</name>
      <anchorfile>classsoar__ros_1_1SoarRunner.html</anchorfile>
      <anchor>a9b579438380e18582d68c66b7a1ab170</anchor>
      <arglist>(const std::shared_ptr&lt; rmw_request_id_t &gt; request_header, std::shared_ptr&lt; std_srvs::srv::Trigger::Request &gt; request, std::shared_ptr&lt; std_srvs::srv::Trigger::Response &gt; response)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>stopSoarKernel</name>
      <anchorfile>classsoar__ros_1_1SoarRunner.html</anchorfile>
      <anchor>a70560c29a2d317f96d29462ac6d44d2a</anchor>
      <arglist>(const std::shared_ptr&lt; rmw_request_id_t &gt; request_header, std::shared_ptr&lt; std_srvs::srv::Trigger::Request &gt; request, std::shared_ptr&lt; std_srvs::srv::Trigger::Response &gt; response)</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>soar_ros::Subscriber</name>
    <filename>classsoar__ros_1_1Subscriber.html</filename>
    <templarg>typename T</templarg>
    <base>soar_ros::Input&lt; T &gt;</base>
    <base>soar_ros::Interface</base>
    <member kind="function">
      <type>void</type>
      <name>subscribe</name>
      <anchorfile>classsoar__ros_1_1Subscriber.html</anchorfile>
      <anchor>a08dfaa0f7fb5afe7018975c1e604561b</anchor>
      <arglist>(rclcpp::Node::SharedPtr node)</arglist>
    </member>
    <member kind="function">
      <type>std::string</type>
      <name>getTopic</name>
      <anchorfile>classsoar__ros_1_1Subscriber.html</anchorfile>
      <anchor>a5cbbcc7f77e12aec0811a4f128a7b871</anchor>
      <arglist>() override</arglist>
    </member>
    <member kind="function">
      <type>sml::Agent *</type>
      <name>getAgent</name>
      <anchorfile>classsoar__ros_1_1Subscriber.html</anchorfile>
      <anchor>ab6ce3f45ea600c5e8e0eef4b85c2a6b1</anchor>
      <arglist>() override</arglist>
    </member>
  </compound>
</tagfile>
