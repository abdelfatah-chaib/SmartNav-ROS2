import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA

class RvizAlertNode(Node):
    def __init__(self):
        super().__init__('rviz_alert_node')
        
        # Abonnement au topic du Binôme 1
        self.subscription = self.create_subscription(
            Int32,
            '/smartnav/alert_level',
            self.alert_callback,
            10)
        
        # Publication du marker RViz
        self.marker_pub = self.create_publisher(Marker, '/smartnav/canne_marker', 10)
        
        # Timer pour le clignotement (10 fois par seconde)
        self.timer = self.create_timer(0.1, self.publish_marker)
        
        self.alert_level = 0
        self.blink_state = True
        self.get_logger().info('Nœud visualisation démarré !')

    def alert_callback(self, msg):
        self.alert_level = msg.data
        if self.alert_level == 0:
            self.get_logger().info('✅ Voie libre')
        elif self.alert_level == 1:
            self.get_logger().warn('⚠️  Attention obstacle éloigné')
        elif self.alert_level == 2:
            self.get_logger().error('🚨 DANGER IMMINENT - Arrêt requis !')

    def publish_marker(self):
        marker = Marker()
        marker.header.frame_id = 'base_link'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'canne'
        marker.id = 0
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD

        # Taille de la canne
        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.scale.z = 1.0

        # Position
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.5
        marker.pose.orientation.w = 1.0

        # Couleur selon le niveau d'alerte
        if self.alert_level == 0:
            # Blanc = voie libre
            marker.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
        elif self.alert_level == 1:
            # Orange = attention
            marker.color = ColorRGBA(r=1.0, g=0.5, b=0.0, a=1.0)
        elif self.alert_level == 2:
            # Rouge clignotant = danger
            self.blink_state = not self.blink_state
            alpha = 1.0 if self.blink_state else 0.0
            marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=alpha)

        self.marker_pub.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    node = RvizAlertNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
