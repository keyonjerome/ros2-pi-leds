import { Component, OnInit } from '@angular/core';
// import * as rclnodejs from 'rclnodejs';

@Component({
  selector: 'app-color-pub',
  templateUrl: './color-pub.component.html',
  styleUrls: ['./color-pub.component.css']
})
export class ColorPubComponent implements OnInit {

  constructor() { }

  ngOnInit(): void {
    // rclnodejs.init().then(() => {
    //   const node = new rclnodejs.Node('publisher_example_node');
    //   const publisher = node.createPublisher('std_msgs/msg/String', 'topic');
    //   publisher.publish(`Hello ROS 2 from rclnodejs`);
    //   node.spin();
    // });
  }

}
