<?xml version='1.0' encoding='UTF-8' standalone='yes' ?>
<tagfile doxygen_version="1.9.1">
  <compound kind="file">
    <name>SafeQueue.hpp</name>
    <path>/home/runner/work/soar_ros/soar_ros/include/soar_ros/</path>
    <filename>SafeQueue_8hpp.html</filename>
    <class kind="class">soar_ros::SafeQueue</class>
  </compound>
  <compound kind="class">
    <name>soar_ros::Client</name>
    <filename>classsoar__ros_1_1Client.html</filename>
    <templarg></templarg>
    <templarg></templarg>
    <templarg></templarg>
    <base virtualness="virtual">Output&lt; typename T::Request::SharedPtr &gt;</base>
    <base virtualness="virtual">Input&lt; typename T::Response::SharedPtr &gt;</base>
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
      <anchor>ae4fe6f9d6c592e42c168d3f1f2c52d27</anchor>
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
    <templarg></templarg>
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
    <name>Input&lt; typename T::Request::SharedPtr &gt;</name>
    <filename>classsoar__ros_1_1Input.html</filename>
    <base>soar_ros::InputBase</base>
    <member kind="function" virtualness="pure">
      <type>virtual void</type>
      <name>parse</name>
      <anchorfile>classsoar__ros_1_1Input.html</anchorfile>
      <anchor>a09a0e5766d8c852f3da30c6bb7723ecf</anchor>
      <arglist>(typename T::Request::SharedPtr msg)=0</arglist>
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
    <name>Input&lt; typename T::Response::SharedPtr &gt;</name>
    <filename>classsoar__ros_1_1Input.html</filename>
    <base>soar_ros::InputBase</base>
    <member kind="function" virtualness="pure">
      <type>virtual void</type>
      <name>parse</name>
      <anchorfile>classsoar__ros_1_1Input.html</anchorfile>
      <anchor>a09a0e5766d8c852f3da30c6bb7723ecf</anchor>
      <arglist>(typename T::Response::SharedPtr msg)=0</arglist>
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
      <anchor>a6f20fe370632f8df09b4cbc79d7cc2a7</anchor>
      <arglist>()=0</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>soar_ros::Output</name>
    <filename>classsoar__ros_1_1Output.html</filename>
    <templarg></templarg>
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
    <name>Output&lt; typename T::Request::SharedPtr &gt;</name>
    <filename>classsoar__ros_1_1Output.html</filename>
    <base>soar_ros::OutputBase</base>
    <member kind="function">
      <type>void</type>
      <name>process_s2r</name>
      <anchorfile>classsoar__ros_1_1Output.html</anchorfile>
      <anchor>ac38d1e23a7093e233209f8ff8f8c9046</anchor>
      <arglist>(sml::Identifier *id) override</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual typename T::Request::SharedPtr</type>
      <name>parse</name>
      <anchorfile>classsoar__ros_1_1Output.html</anchorfile>
      <anchor>af01db0104f6bd68b61ac9baff164926f</anchor>
      <arglist>(sml::Identifier *id)=0</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>Output&lt; typename T::Response::SharedPtr &gt;</name>
    <filename>classsoar__ros_1_1Output.html</filename>
    <base>soar_ros::OutputBase</base>
    <member kind="function">
      <type>void</type>
      <name>process_s2r</name>
      <anchorfile>classsoar__ros_1_1Output.html</anchorfile>
      <anchor>ac38d1e23a7093e233209f8ff8f8c9046</anchor>
      <arglist>(sml::Identifier *id) override</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual typename T::Response::SharedPtr</type>
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
    <templarg></templarg>
    <base>soar_ros::Output</base>
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
      <anchor>a32b5a0549e5eadce8483db3f359e4d37</anchor>
      <arglist>() override</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>soar_ros::SafeQueue</name>
    <filename>classsoar__ros_1_1SafeQueue.html</filename>
    <templarg></templarg>
  </compound>
  <compound kind="class">
    <name>soar_ros::Service</name>
    <filename>classsoar__ros_1_1Service.html</filename>
    <templarg></templarg>
    <templarg></templarg>
    <templarg></templarg>
    <base virtualness="virtual">Input&lt; typename T::Request::SharedPtr &gt;</base>
    <base virtualness="virtual">Output&lt; typename T::Response::SharedPtr &gt;</base>
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
      <anchor>aedb3ff0a398aa821369e3a6521a70fab</anchor>
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
      <anchor>a8e758f86ce719f1035852e85bfff9a2d</anchor>
      <arglist>([[maybe_unused]] const std::shared_ptr&lt; rmw_request_id_t &gt; request_header, [[maybe_unused]] std::shared_ptr&lt; std_srvs::srv::Trigger::Request &gt; request, std::shared_ptr&lt; std_srvs::srv::Trigger::Response &gt; response)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>getSoarKernelStatus</name>
      <anchorfile>classsoar__ros_1_1SoarRunner.html</anchorfile>
      <anchor>af60fee9c9b50e0696f7fabd444b441b5</anchor>
      <arglist>([[maybe_unused]] const std::shared_ptr&lt; rmw_request_id_t &gt; request_header, [[maybe_unused]] std::shared_ptr&lt; std_srvs::srv::Trigger::Request &gt; request, std::shared_ptr&lt; std_srvs::srv::Trigger::Response &gt; response)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>runSoarKernel</name>
      <anchorfile>classsoar__ros_1_1SoarRunner.html</anchorfile>
      <anchor>ad615385cb571b0859fa341a347ba1a79</anchor>
      <arglist>([[maybe_unused]] const std::shared_ptr&lt; rmw_request_id_t &gt; request_header, [[maybe_unused]] std::shared_ptr&lt; std_srvs::srv::Trigger::Request &gt; request, std::shared_ptr&lt; std_srvs::srv::Trigger::Response &gt; response)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>stopSoarKernel</name>
      <anchorfile>classsoar__ros_1_1SoarRunner.html</anchorfile>
      <anchor>a65d354c261d99756222af4a790981e58</anchor>
      <arglist>([[maybe_unused]] const std::shared_ptr&lt; rmw_request_id_t &gt; request_header, [[maybe_unused]] std::shared_ptr&lt; std_srvs::srv::Trigger::Request &gt; request, std::shared_ptr&lt; std_srvs::srv::Trigger::Response &gt; response)</arglist>
    </member>
    <member kind="function">
      <type>sml::Agent *</type>
      <name>addAgent</name>
      <anchorfile>classsoar__ros_1_1SoarRunner.html</anchorfile>
      <anchor>a130cdf890594f230d7e987394c93e002</anchor>
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
      <anchor>ad773ed65ead9c40e883ffadbd314a346</anchor>
      <arglist>(std::shared_ptr&lt; soar_ros::Publisher&lt; T &gt;&gt; output)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>addPublisher</name>
      <anchorfile>classsoar__ros_1_1SoarRunner.html</anchorfile>
      <anchor>a010eff1a3de5ae0eed9078225773e669</anchor>
      <arglist>(std::shared_ptr&lt; soar_ros::Publisher&lt; T &gt;&gt; output, const std::string &amp;commandName)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>addService</name>
      <anchorfile>classsoar__ros_1_1SoarRunner.html</anchorfile>
      <anchor>acb4c15839939a62cb4554e5bb8118e77</anchor>
      <arglist>(std::shared_ptr&lt; soar_ros::Service&lt; T &gt;&gt; service)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>addService</name>
      <anchorfile>classsoar__ros_1_1SoarRunner.html</anchorfile>
      <anchor>acf0876f03bae81626d9e3649162cc13c</anchor>
      <arglist>(std::shared_ptr&lt; soar_ros::Service&lt; T &gt;&gt; service, const std::string &amp;commandName)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>addClient</name>
      <anchorfile>classsoar__ros_1_1SoarRunner.html</anchorfile>
      <anchor>a6b92de426f03f7ed27729d6ff5a7a726</anchor>
      <arglist>(std::shared_ptr&lt; soar_ros::Client&lt; T &gt;&gt; client)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>addClient</name>
      <anchorfile>classsoar__ros_1_1SoarRunner.html</anchorfile>
      <anchor>aba2ffb76aca69fc5441c6a90e3a7a910</anchor>
      <arglist>(std::shared_ptr&lt; soar_ros::Client&lt; T &gt;&gt; client, const std::string &amp;commandName)</arglist>
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
    <templarg></templarg>
    <base>soar_ros::Input</base>
    <base>soar_ros::Interface</base>
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
      <anchor>aa5b8d2ffaa6264e9eb3f5a11e1c76334</anchor>
      <arglist>() override</arglist>
    </member>
  </compound>
</tagfile>
