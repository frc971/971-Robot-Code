if /jevois/deploy/serial_waiter; then
  /jevois/deploy/target_sender &> /jevois/log.log
fi

touch /tmp/do_not_export_sd_card
