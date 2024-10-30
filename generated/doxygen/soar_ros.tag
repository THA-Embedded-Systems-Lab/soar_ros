<?xml version='1.0' encoding='UTF-8' standalone='yes' ?>
<tagfile doxygen_version="1.12.0" doxygen_gitid="c73f5d30f9e8b1df5ba15a1d064ff2067cbb8267">
  <compound kind="file">
    <name>SafeQueue.hpp</name>
    <path>include/soar_ros/</path>
    <filename>SafeQueue_8hpp.html</filename>
    <class kind="class">soar_ros::soar_ros::SafeQueue</class>
  </compound>
  <compound kind="class">
    <name>soar_ros::Client</name>
    <filename>classsoar__ros_1_1Client.html</filename>
    <templarg>typename T</templarg>
    <templarg>typename pRequestType</templarg>
    <templarg>typename pResponseType</templarg>
    <base virtualness="virtual">soar_ros::soar_ros::Output&lt; typename T::Request::SharedPtr &gt;</base>
    <base virtualness="virtual">soar_ros::soar_ros::Input&lt; typename T::Response::SharedPtr &gt;</base>
    <base>soar_ros::soar_ros::Interface</base>
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
    <name>soar_ros::soar_ros::Input</name>
    <filename>classsoar__ros_1_1soar__ros_1_1Input.html</filename>
    <templarg>typename T</templarg>
    <base>soar_ros::soar_ros::InputBase</base>
    <member kind="function" virtualness="pure">
      <type>virtual void</type>
      <name>parse</name>
      <anchorfile>classsoar__ros_1_1soar__ros_1_1Input.html</anchorfile>
      <anchor>a6b2c746a395c5290d3338d55e0367fed</anchor>
      <arglist>(T msg)=0</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>process_r2s</name>
      <anchorfile>classsoar__ros_1_1soar__ros_1_1Input.html</anchorfile>
      <anchor>af0d5c1222d36d8bc84ab3f6b719b8492</anchor>
      <arglist>() override</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>soar_ros::soar_ros::InputBase</name>
    <filename>classsoar__ros_1_1soar__ros_1_1InputBase.html</filename>
    <member kind="function" virtualness="pure">
      <type>virtual void</type>
      <name>process_r2s</name>
      <anchorfile>classsoar__ros_1_1soar__ros_1_1InputBase.html</anchorfile>
      <anchor>af350a7fc94716effcf2e17bb540fc125</anchor>
      <arglist>()=0</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>soar_ros::soar_ros::Interface</name>
    <filename>classsoar__ros_1_1soar__ros_1_1Interface.html</filename>
    <member kind="function" virtualness="pure">
      <type>virtual std::string</type>
      <name>getTopic</name>
      <anchorfile>classsoar__ros_1_1soar__ros_1_1Interface.html</anchorfile>
      <anchor>ac9e494efcf52d9a574bbb13d4275dff5</anchor>
      <arglist>()=0</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual sml::Agent *</type>
      <name>getAgent</name>
      <anchorfile>classsoar__ros_1_1soar__ros_1_1Interface.html</anchorfile>
      <anchor>a3913469a81fd03c62f1683c48aed8923</anchor>
      <arglist>()=0</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>soar_ros::soar_ros::Output</name>
    <filename>classsoar__ros_1_1soar__ros_1_1Output.html</filename>
    <templarg>typename T</templarg>
    <base>soar_ros::soar_ros::OutputBase</base>
    <member kind="function">
      <type>void</type>
      <name>process_s2r</name>
      <anchorfile>classsoar__ros_1_1soar__ros_1_1Output.html</anchorfile>
      <anchor>a30db5b8255ba15c91a388496050b3fac</anchor>
      <arglist>(sml::Identifier *id) override</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual T</type>
      <name>parse</name>
      <anchorfile>classsoar__ros_1_1soar__ros_1_1Output.html</anchorfile>
      <anchor>a728f759fd5538bf8c21e52ad9d387742</anchor>
      <arglist>(sml::Identifier *id)=0</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>soar_ros::soar_ros::OutputBase</name>
    <filename>classsoar__ros_1_1soar__ros_1_1OutputBase.html</filename>
    <member kind="function" virtualness="pure">
      <type>virtual void</type>
      <name>process_s2r</name>
      <anchorfile>classsoar__ros_1_1soar__ros_1_1OutputBase.html</anchorfile>
      <anchor>a14fe8c99313410a1833c2e3dffa6f893</anchor>
      <arglist>(sml::Identifier *id)=0</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>soar_ros::Publisher</name>
    <filename>classsoar__ros_1_1Publisher.html</filename>
    <templarg>typename T</templarg>
    <base>soar_ros::soar_ros::Output&lt; T &gt;</base>
    <base>soar_ros::soar_ros::Interface</base>
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
    <name>soar_ros::soar_ros::SafeQueue</name>
    <filename>classsoar__ros_1_1soar__ros_1_1SafeQueue.html</filename>
    <templarg>typename T</templarg>
  </compound>
  <compound kind="class">
    <name>soar_ros::Service</name>
    <filename>classsoar__ros_1_1Service.html</filename>
    <templarg>typename T</templarg>
    <templarg>typename pRequestType</templarg>
    <templarg>typename pResponseType</templarg>
    <base virtualness="virtual">soar_ros::soar_ros::Input&lt; typename T::Request::SharedPtr &gt;</base>
    <base virtualness="virtual">soar_ros::soar_ros::Output&lt; typename T::Response::SharedPtr &gt;</base>
    <base>soar_ros::soar_ros::Interface</base>
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
    <name>soar_ros::SoarRunner</name>
    <filename>classsoar__ros_1_1SoarRunner.html</filename>
    <member kind="function">
      <type></type>
      <name>SoarRunner</name>
      <anchorfile>classsoar__ros_1_1SoarRunner.html</anchorfile>
      <anchor>a7f582f9e83635a872f0b08f6c8046088</anchor>
      <arglist>(const std::string &amp;agent_name, const std::string &amp;path_productions)</arglist>
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
    <member kind="function">
      <type>sml::Agent *</type>
      <name>addAgent</name>
      <anchorfile>classsoar__ros_1_1SoarRunner.html</anchorfile>
      <anchor>a25a39e1fe1dbb815a7781caa9d63895b</anchor>
      <arglist>(const std::string &amp;agent_name, const std::string &amp;path_productions)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>~SoarRunner</name>
      <anchorfile>classsoar__ros_1_1SoarRunner.html</anchorfile>
      <anchor>a99a8e825b46bdf372ca99c1632c9d8ce</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>addPublisher</name>
      <anchorfile>classsoar__ros_1_1SoarRunner.html</anchorfile>
      <anchor>a5c03ccf5186a8386b73ed23f933ce492</anchor>
      <arglist>(std::shared_ptr&lt; soar_ros::Publisher&lt; T &gt; &gt; output)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>addPublisher</name>
      <anchorfile>classsoar__ros_1_1SoarRunner.html</anchorfile>
      <anchor>a3d62f1328da585cbb7c55010a31352a4</anchor>
      <arglist>(std::shared_ptr&lt; soar_ros::Publisher&lt; T &gt; &gt; output, const std::string &amp;commandName)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>addService</name>
      <anchorfile>classsoar__ros_1_1SoarRunner.html</anchorfile>
      <anchor>abbadbd6a39c3dbb7e4490cac11965a29</anchor>
      <arglist>(std::shared_ptr&lt; soar_ros::Service&lt; T &gt; &gt; service)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>addService</name>
      <anchorfile>classsoar__ros_1_1SoarRunner.html</anchorfile>
      <anchor>a0edd9b8aadb1440f363a316eee091dd6</anchor>
      <arglist>(std::shared_ptr&lt; soar_ros::Service&lt; T &gt; &gt; service, const std::string &amp;commandName)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>addClient</name>
      <anchorfile>classsoar__ros_1_1SoarRunner.html</anchorfile>
      <anchor>ac47e59d8073947e53a213bb12b130e67</anchor>
      <arglist>(std::shared_ptr&lt; soar_ros::Client&lt; T &gt; &gt; client)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>addClient</name>
      <anchorfile>classsoar__ros_1_1SoarRunner.html</anchorfile>
      <anchor>a12ee7e8dceadb4ca8aaa55262839d3e3</anchor>
      <arglist>(std::shared_ptr&lt; soar_ros::Client&lt; T &gt; &gt; client, const std::string &amp;commandName)</arglist>
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
  </compound>
  <compound kind="class">
    <name>soar_ros::Subscriber</name>
    <filename>classsoar__ros_1_1Subscriber.html</filename>
    <templarg>typename T</templarg>
    <base>soar_ros::soar_ros::Input&lt; T &gt;</base>
    <base>soar_ros::soar_ros::Interface</base>
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
