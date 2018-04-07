
in ie:/etc# cat sysctl.conf 
set to 1 the following flag:
# Uncomment the next line to enable packet forwarding for IPv4
net.ipv4.ip_forward=1


 cat > /etc/iptables.v4
*nat
-A POSTROUTING -o eth0 -j MASQUERADE
COMMIT

END

add to /etc/rc.local:
iptables-restore < /etc/iptables.v4




create /etc/network/interfaces.d/eth1 with:
allow-hotplug eth1
iface eth1 inet static
        address 11.0.0.179
        netmask 255.0.0.0
