#!/usr/bin/python

import krpc

def setup(conn):
  buttons={}
  clicked={}
  canvas = conn.ui.stock_canvas

  # Get the size of the game window in pixels
  screen_size = canvas.rect_transform.size

  # Add a panel to contain the UI elements
  panel = canvas.add_panel()

  # Position the panel on the left of the screen
  rect = panel.rect_transform
  rect.size = (200, 160)
  rect.position = (110-(screen_size[0]/2), 0)

  # Add a button to set the throttle to maximum
  buttons['compute'] = panel.add_button("Compute Trajectory")
  buttons['compute'].rect_transform.position = (0, 20)
  clicked['compute'] = conn.add_stream(getattr, buttons['compute'], 'clicked')

  # Change target
  buttons['change'] = panel.add_button("Change target")
  buttons['change'].rect_transform.position = (0, -20)
  clicked['change'] = conn.add_stream(getattr, buttons['change'], 'clicked')

  # Add some text displaying the total engine thrust
  text = panel.add_text("")
  text.rect_transform.position = (0, -60)
  text.color = (1, 1, 1)
  text.size = 18
  buttons['target'] = text

  # Autopilot ON
  buttons['autoOn'] = panel.add_button("Autopilot ON")
  buttons['autoOn'].rect_transform.position = (0, -100)
  clicked['autoOn'] = conn.add_stream(getattr, buttons['autoOn'], 'clicked')

  # Autopilot OFF
  buttons['autoOff'] = panel.add_button("Autopilot OFF")
  buttons['autoOff'].rect_transform.position = (0, -140)
  clicked['autoOff'] = conn.add_stream(getattr, buttons['autoOff'], 'clicked')

  # Emergency land
  buttons['emergency'] = panel.add_button("Emergency Land")
  buttons['emergency'].rect_transform.position = (0, -180)
  clicked['emergency'] = conn.add_stream(getattr, buttons['emergency'], 'clicked')

  return buttons,clicked
