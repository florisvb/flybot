import numpy as np
import matplotlib.pyplot as plt

def plot_im(im, cmap="gray"):
    fig = plt.figure()
    ax = fig.add_axes([0, 0, 1, 1])
    
    ax.spines['right'].set_color('none')
    ax.spines['left'].set_color('none')
    ax.spines['top'].set_color('none')
    ax.spines['bottom'].set_color('none')
    ax.xaxis.set_ticks_position('none')
    ax.yaxis.set_ticks_position('none')
    ax.xaxis.set_ticklabels([])
    ax.yaxis.set_ticklabels([])
    
    ax.imshow(image, cmap=cmap)
    plt.show()


if 1:
    frequencies = [.1, .5, .55, 1, 1.5, 10, 15]
    amplitudes = [1,1,1,1,1,1,1,1]

if 0:
    frequencies = [10,15]
    amplitudes = [1,1]

image_aspect_ratio = [3,2]
image_length_pixels = 5000
image_size_pixels = [image_length_pixels, int(image_length_pixels/float(image_aspect_ratio[0])*float(image_aspect_ratio[1]))]

t = np.linspace(0,np.pi*10,image_size_pixels[0])

sin_sum = np.zeros_like(t)

for i, f in enumerate(frequencies):
    a = amplitudes[i]
    sin_sum += a*np.sin(2*np.pi*t*f)
    
image = np.tile(sin_sum, (image_size_pixels[1], 1))
    
plot_im(image, cmap="gray")





