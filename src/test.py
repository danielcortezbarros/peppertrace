from pynput import mouse

def on_scroll(x, y, dx, dy):

    
    # Stop the listener when the scroll event is detected
    return False

def on_click(x, y, button, pressed):
    if pressed:
        if button == mouse.Button.left:
            print("apple", flush=True)
        elif button == mouse.Button.right:
            print("banana", flush=True)

# Start listening for both click and scroll events
with mouse.Listener(on_click=on_click, on_scroll=on_scroll) as listener:
    listener.join()
