import numpy as np
import matplotlib.pyplot as plt

# Visualize convolution. See https://tonysyu.github.io/
def iter_pixels(image):
    """ Yield pixel position (row, column) and pixel intensity. """
    height, width = image.shape[:2]
    for i in range(height):
        for j in range(width):
            yield (i, j), image[i, j]


# Visualize result
def imshow_pair(image_pair, titles=('', ''), figsize=(10, 5), **kwargs):
    fig, axes = plt.subplots(ncols=2, figsize=figsize)
    for ax, img, label in zip(axes.ravel(), image_pair, titles):
        ax.imshow(img, **kwargs)
        ax.set_title(label)


# Visualize result
def imshow_triple(axes, image_pair, titles=('', '', ''), figsize=(10, 5), **kwargs):
    for ax, img, label in zip(axes, image_pair, titles):
        ax.imshow(img, **kwargs)
        ax.set_title(label, fontdict={'fontsize': 8})
        ax.set_xticks([])
        ax.set_yticks([])


# Zero-padding
def padding_for_kernel(kernel):
    """ Return the amount of padding needed for each side of an image.
    For example, if the returned result is [1, 2], then this means an
    image should be padded with 1 extra row on top and bottom, and 2
    extra columns on the left and right.
    """
    # Slice to ignore RGB channels if they exist.
    image_shape = kernel.shape[:2]
    # We only handle kernels with odd dimensions so make sure that's true.
    # (The "center" pixel of an even number of pixels is arbitrary.)
    assert all((size % 2) == 1 for size in image_shape)
    return [(size - 1) // 2 for size in image_shape]


def add_padding(image, kernel):
    h_pad, w_pad = padding_for_kernel(kernel)
    return np.pad(image, ((h_pad, h_pad), (w_pad, w_pad)),
                  mode='constant', constant_values=0)


def remove_padding(image, kernel):
    inner_region = []  # A 2D slice for grabbing the inner image region
    for pad in padding_for_kernel(kernel):
        slice_i = slice(None) if pad == 0 else slice(pad, -pad)
        inner_region.append(slice_i)
    return image[tuple(inner_region)]


# Slice windows
def window_slice(center, kernel):
    r, c = center
    r_pad, c_pad = padding_for_kernel(kernel)
    # Slicing is (inclusive, exclusive) so add 1 to the stop value
    return [slice(r - r_pad, r + r_pad + 1), slice(c - c_pad, c + c_pad + 1)]


# Apply convolution kernel to image patch
def apply_kernel(center, kernel, original_image):
    image_patch = original_image[tuple(window_slice(center, kernel))]
    # An element-wise multiplication followed by the sum
    return np.sum(kernel * image_patch)


# Move kernel over the image
def iter_kernel_labels(image, kernel):
    original_image = image
    image = add_padding(original_image, kernel)
    i_pad, j_pad = padding_for_kernel(kernel)

    for (i, j), pixel in iter_pixels(original_image):
        # Shift the center of the kernel to ignore padded border.
        i += i_pad
        j += j_pad
        mask = np.zeros(image.shape, dtype=int)  # Background = 0
        mask[tuple(window_slice((i, j), kernel))] = kernel  # Kernel = 1
        # mask[i, j] = 2                           # Kernel-center = 2
        yield (i, j), mask


# Visualize kernel as it moves over the image
def visualize_kernel(kernel_labels, image):
    return kernel_labels + image  # color.label2rgb(kernel_labels, image, bg_label=0)


# Do a single step
def convolution_demo(image, kernel, **kwargs):
    # Initialize generator since we're only ever going to iterate over
    # a pixel once. The cached result is used, if we step back.
    gen_kernel_labels = iter_kernel_labels(image, kernel)

    image_cache = []
    image_padded = add_padding(image, kernel)

    # Plot original image and kernel-overlay next to filtered image.
    def convolution_step(i_step=0):
        # Create all images up to the current step, unless they're already
        # cached:
        while i_step >= len(image_cache):
            # For the first step (`i_step == 0`), the original image is the
            # filtered image; after that we look in the cache, which stores
            # (`kernel_overlay`, `filtered`).
            filtered_prev = image_padded if i_step == 0 else image_cache[-1][1]
            # We don't want to overwrite the previously filtered image:
            filtered = filtered_prev.copy()

            # Get the labels used to visualize the kernel
            center, kernel_labels = next(gen_kernel_labels)
            # Modify the pixel value at the kernel center
            filtered[center] = apply_kernel(center, kernel, image_padded)
            # Take the original image and overlay our kernel visualization
            kernel_overlay = visualize_kernel(kernel_labels, image_padded)
            # Save images for reuse.
            image_cache.append((kernel_overlay, filtered))

        # Remove padding we added to deal with boundary conditions
        # (Loop since each step has 2 images)
        image_pair = [remove_padding(each, kernel)
                      for each in image_cache[i_step]]
        imshow_pair(image_pair, **kwargs)

    return convolution_step


# Full process
def convolution_full(ax, image, kernel, **kwargs):
    # Initialize generator since we're only ever going to iterate over
    # a pixel once. The cached result is used, if we step back.
    gen_kernel_labels = iter_kernel_labels(image, kernel)

    image_cache = []
    image_padded = add_padding(image, kernel)
    # Plot original image and kernel-overlay next to filtered image.

    for i_step in range(image.size - 1):
        # For the first step (`i_step == 0`), the original image is the
        # filtered image; after that we look in the cache, which stores
        # (`kernel_overlay`, `filtered`).
        filtered_prev = image_padded if i_step == 0 else image_cache[-1][1]
        # We don't want to overwrite the previously filtered image:
        filtered = filtered_prev.copy()

        # Get the labels used to visualize the kernel
        center, kernel_labels = next(gen_kernel_labels)
        # Modify the pixel value at the kernel center
        filtered[center] = apply_kernel(center, kernel, image_padded)
        # Take the original image and overlay our kernel visualization
        kernel_overlay = visualize_kernel(kernel_labels, image_padded)
        # Save images for reuse.
        image_cache.append((kernel_overlay, filtered))

    # Remove padding we added to deal with boundary conditions
    # (Loop since each step has 2 images)
    image_triple = [remove_padding(each, kernel)
                    for each in image_cache[i_step]]
    image_triple.insert(1, kernel)

    ax.imshow(image_triple[2], aspect='auto', **kwargs)
    ax.set_xticks([])
    ax.set_yticks([])

## Credits: Convolutional Neural Networks, Joaquin Vanschoren, Eindhoven University of Technology
