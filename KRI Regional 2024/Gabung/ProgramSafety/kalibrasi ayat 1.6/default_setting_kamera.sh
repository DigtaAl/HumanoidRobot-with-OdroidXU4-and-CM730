echo Before V4l setting
v4l2-ctl --list-ctrls

        #constant setting fitur camera
        v4l2-ctl --set-ctrl focus_auto=0
        v4l2-ctl --set-ctrl white_balance_temperature_auto=0
        v4l2-ctl --set-ctrl white_balance_temperature=4650
        v4l2-ctl --set-ctrl exposure_auto_priority=0
        v4l2-ctl --set-ctrl exposure_auto=3
        v4l2-ctl --set-ctrl exposure_absolute=166

        v4l2-ctl --set-ctrl brightness=0
        v4l2-ctl --set-ctrl contrast=32
        v4l2-ctl --set-ctrl saturation=64
        v4l2-ctl --set-ctrl sharpness=4

echo After value
v4l2-ctl --list-ctrls
