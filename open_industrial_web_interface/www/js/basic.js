
/**
 * Setup all visualization elements when the page is loaded. 
 */

var fixed_frame;
var current_group;
var link_group;
var end_effector_link;
var initial_flag = true;
var tfClient;

function init() {
    // Connect to ROS.
    var url = 'ws://' + location.hostname + ':9090';

    var real_ros = new ROSLIB.Ros({
        url : url
    });
    var virtual_ros = new ROSLIB.Ros({
        url : url
    });

    var joint_ros = new ROSLIB.Ros({
        url : url
    });

    joint_names = new ROSLIB.Param({
        ros: joint_ros,
        name: '/joint'
    });

    pub = new ROSLIB.Topic({
        ros: joint_ros,
        name: '/virtual_update_joint_position',
        messageType: 'std_msgs/Float64MultiArray'
    });

    moveit_pub = new ROSLIB.Topic({
        ros: joint_ros,
        name: '/moveit_joint',
        messageType: 'open_industrial_web_interface/MoveGroupPlan'
    });

    execute_pub = new ROSLIB.Topic({
        ros: joint_ros,
        name: '/execute_trajectory',
        messageType: 'std_msgs/Empty'
    });


    joint_pub = new ROSLIB.Topic({
        ros: joint_ros,
        name: '/update_joint_position',
        messageType: 'std_msgs/Float64MultiArray'
    });

    computefkClient = new ROSLIB.Service({
        ros : joint_ros,
        name : '/compute_fk',
        serviceType : 'moveit_msgs/GetPositionFK'
    });

    initial_interactive_pub = new ROSLIB.Topic({
        ros: joint_ros,
        name: '/initial_marker',
        messageType: 'std_msgs/String'
    });

    interactive_pub = new ROSLIB.Topic({
        ros: joint_ros,
        name: '/basic_marker/feedback',
        messageType: 'visualization_msgs/InteractiveMarkerFeedback'
    });

    plan_listener = new ROSLIB.Topic({
        ros: joint_ros,
        name: '/stock_joint_position',
        messageType: 'std_msgs/Float64MultiArray'
    });


    // Create the main viewer.
    var width = parseInt($("#main-content").css("width"));
    var height = document.height;

    var viewer = new ROS3D.Viewer({
        divID : 'urdf',
        width : width * 0.8,
        height : height * 0.8,
        antialias : true
    });

    // Add a grid.
    viewer.addObject(new ROS3D.Grid());


    fixed_frame_param = new ROSLIB.Param({
        ros: real_ros,
        name: '/fixed_frame'
    });

    link_group_param = new ROSLIB.Param({
        ros: virtual_ros,
        name: '/link_group/'
    });

    end_effector_link_param = new ROSLIB.Param({
        ros: joint_ros,
        name: '/end_effector_link/'
    });

    // Setup listener
    var listener = new ROSLIB.Topic({
        ros : virtual_ros,
        name : '/virtual_joint_states',
        messageType : 'sensor_msgs/JointState'
    });


    // Setup a client to listen to TFs.
    fixed_frame_param.get(function(value) {
        fixed_frame = value;
        tfClient = new ROSLIB.TFClient({
            ros : real_ros,
            fixedFrame : fixed_frame,
            angularThres : 0.01,
            transThres : 0.01,
            rate : 10.0
        });
        // Setup the marker client.
        var imClient = new ROS3D.InteractiveMarkerClient({
            ros : real_ros,
            tfClient : tfClient,
            topic : '/basic_marker',
            camera : viewer.camera,
            rootObject : viewer.selectableObjects
        });
    });

    link_group_param.get(function(value) {
        link_group = value;
        for (group_name in link_group) {
            $('#group').append("<option value=" + group_name + ">" + group_name + "</option>");
        }
        $("select#group").bind('change', function() {
            var selector = $("select#group option");
            selector.each(function() {
                $("#" + $(this).val()).hide();
            });
            var group = $("select#group option:selected").val();
            current_group = group;
            $("#" + group).show();
            var msg = new ROSLIB.Message({
                data: current_group
            });
            initial_interactive_pub.publish(msg);
            create_joint_position_msg(1, true);
        });
        setTimeout(function() {
            createSliderView();

            listener.subscribe(function(message) {
                for (i = 0; i < message.name.length; i++) {
                    var joint_name, joint_num;
                
                    for (j = 0; j < link_group[current_group].length; j++) {
                        if (link_group[current_group][j] == message.name[i]) {
                            var min = $('input#' + link_group[current_group][j]).attr("min");
                            var max = $('input#' + link_group[current_group][j]).attr("max");
                            var percent = parseInt((message.position[i] - min)/(max - min) * 100);
                            $('input#' + link_group[current_group][j]).attr("value", message.position[i]);
                            $('input#' + link_group[current_group][j]).next().children("div").attr("style", "width: " + percent + "%;");
                            $('input#' + link_group[current_group][j]).next().children("a").attr("aria-valuenow", message.position[i]);
                            $('input#' + link_group[current_group][j]).next().children("a").attr("aria-valuetext", 0.5);
                            $('input#' + link_group[current_group][j]).next().children("a").attr("title", message.position[i]);
                            $('input#' + link_group[current_group][j]).next().children("a").attr("style", "left: " +  percent + "%;");
                            $('input#' + link_group[current_group][j]).attr("value", message.position[i]);
                            break;
                        }
                    }
                }
            });

            create_joint_position_msg(1, true);
        }, 3000);

        setTimeout(function() {
            // Setup the URDF client.
            var virtualUrdfClient = new ROS3D.UrdfClient({
                ros : virtual_ros,
                tfPrefix : 'virtual',
                color : 0xff3000,
                tfClient : tfClient,
                param : 'robot_description',
                rootObject : viewer.scene
            });


            var urdfClient = new ROS3D.UrdfClient({
                ros : real_ros,
                tfClient : tfClient,
                param : 'robot_description',
                rootObject : viewer.scene
            });
        }, 1500);
    });

    end_effector_link_param.get(function(value) {
        end_effector_link = value;
    });

    plan_listener.subscribe(function(message) {
        message_stock.push(message);
    });

    $("button#moveit").click(function() {
        var msg = create_joint_position_msg(0,false);
        moveit_pub.publish(msg);
    });

    $("button#plan").click(function() {
        message_stock = new Array();
        var msg = create_joint_position_msg(0,true);
        moveit_pub.publish(msg);
    });

    $("button#execute").click(function() {
        if(message_stock != null) {
            sim_mode = new ROSLIB.Param({
                ros: joint_ros,
                name: '/sim_mode'
            });
            sim_mode.get(function(value) {
                if (value == true) {
                    timer = setInterval("joint_publish()",100);
                }
                else {
                    var msg = new ROSLIB.Message({
                    });
                    execute_pub.publish(msg);
                }                
            });
        }
    });
}

function create_joint_position_msg(type, plan_only) {

    positions = new Array();
    joints = new Array();
    dims = new Array();

    $("#" + current_group).children("label").each(function() {
        var dim = new ROSLIB.Message({
            label: ($(this).attr("id").split("-")[0]),
            size: ($(this).attr("id").split("-")[0]).length,
            stride: ($(this).attr("id").split("-")[0]).length
        });
        dims.push(dim);
        joints.push($(this).attr("id").split("-")[0]);
        positions.push(parseFloat($(this).next().children("input").next().children("a").attr("aria-valuenow")));
    });

    var msg;
    if (type == 0) {
        msg = new ROSLIB.Message({
            joint: {
                layout: {
                    dim: dims,
                    data_offset: 0
                },
                data: positions
            },
            plan_only: plan_only,
            group_name: current_group
        });
    }
    else {
        msg = new ROSLIB.Message({
            layout: {
                dim: dims,
                data_offset: 0
            },
            data: positions
        });
    }
    var fk_link_name;

    if (end_effector_link[current_group] == null) {
        estimated_link_name = joints[joints.length - 1];
        fk_link_name = estimated_link_name.charAt(0).toUpperCase() + estimated_link_name.slice(1);
    }
    else {
        fk_link_name = end_effector_link[current_group];
    }

    // Update interactive marker poisition
    var request = new ROSLIB.ServiceRequest({
        header: {
            seq: 0,
            stamp: 0,
            frame_id: fixed_frame
        },
        fk_link_names: [fk_link_name],
        robot_state: {
            joint_state: {
                name: joints,
                position: positions
            }
        }
    });

    computefkClient.callService(request, function(result) {

        var interactive_msg = new ROSLIB.Message({
            marker_name: "basic",
            event_type: 5,
            pose: {
                position: {
                    x: result.pose_stamped[0].pose.position.x,
                    y: result.pose_stamped[0].pose.position.y,
                    z: result.pose_stamped[0].pose.position.z
                },
                orientation: {
                    x: result.pose_stamped[0].pose.orientation.x,
                    y: result.pose_stamped[0].pose.orientation.y,
                    z: result.pose_stamped[0].pose.orientation.z,
                    w: result.pose_stamped[0].pose.orientation.w
                }
            }
        });
        interactive_pub.publish(interactive_msg);
    });
    return msg;
}

function callback() {
    var msg = create_joint_position_msg(1, true);
    pub.publish(msg);
}

function joint_publish() {
    if(message_stock.length == 0) {
        clearInterval(timer);
    }
    else {
        joint_pub.publish(message_stock.shift());
    }
}

// create joint_publisher
function createSliderView() {
    var i = 0;
    for (group_name in link_group) {
        $("#slider-pane").append('<div id="' + group_name + '"/>');
        if (i != 0) {
            $("#" + group_name).hide();
        }
        else {
            current_group = group_name;
            i++;
        }
    }
    joint_names.get(function(value) {
        names = value.names;
        for (group_name in link_group) {
            for (var i = 0;i < names.length;i++) {
                if (link_group[group_name].indexOf(names[i]) != -1) {
                    child = $('<label>', {for: names[i], text: names[i]});
                    child2 = $('<input>', {type: "range", name: names[i], id: names[i], value: 0, max: eval("value." + names[i] + ".max"), min: eval("value." + names[i] + ".min"), step: 0.000001, "data-highlight": "true", "data-mini": "true",
                                   onchange: "callback()"});
                    $("#" + group_name).append(child);
                    $("#" + group_name).append(child2);
                }
            }
        }
        $.getScript("js/jquery-mobile/jquery.mobile-1.3.2.min.js");
        var msg = new ROSLIB.Message({
            data: current_group
        });
        initial_interactive_pub.publish(msg);

    });
}
