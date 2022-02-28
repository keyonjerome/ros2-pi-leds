import { Component, OnInit } from '@angular/core';
// https://github.com/RobotWebTools/rclnodejs

// const rclnodejs = require('rclnodejs');
@Component({
  selector: 'app-root',
  templateUrl: './app.component.html',
  styleUrls: ['./app.component.css']
})
export class AppComponent implements OnInit {
  title = 'ros2-smart-home';

  constructor() {

  }

  ngOnInit(): void {
          // rclnodejs.init().then(() => {
    //   const node = new rclnodejs.Node('publisher_example_node');
    //   const publisher = node.createPublisher('std_msgs/msg/String', 'topic');
    //   publisher.publish(`Hello ROS 2 from rclnodejs`);
    //   node.spin();
    // });


  }
}
