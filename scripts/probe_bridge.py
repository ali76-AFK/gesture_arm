import roslibpy, time
c = roslibpy.Ros(host="localhost", port=9090)
c.run(timeout=15)
print("connected:", c.is_connected)
got = []
sub = roslibpy.Topic(c, "/probe_back", "std_msgs/String"); sub.subscribe(lambda m: got.append(m["data"]))
pub = roslibpy.Topic(c, "/probe_msg", "std_msgs/String"); pub.advertise()
time.sleep(1)
for i in range(5):
    pub.publish(roslibpy.Message({"data": f"hello {i}"})); time.sleep(0.4)
time.sleep(2)
print("roslibpy received from ROS:", len(got), got[:2])
c.terminate()
