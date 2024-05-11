import cv2
import ffmpeg
import numpy as np

ffmpeg_input = '/dev/video2'
video_stream = (
    ffmpeg
    .input(ffmpeg_input,s="3840x2160",pix_fmt="nv12")
    .output('pipe:', format='rawvideo', pix_fmt='nv12')
    .run_async(pipe_stdout=True)
)

cv2.namedWindow("result", 0)
cv2.resizeWindow("result", (800, 450))

while True:
    in_bytes = video_stream.stdout.read(3840 * 2160 * 3 // 2)  # nv12 format
    if not in_bytes:
        break
    in_frame = (
        np
        .frombuffer(in_bytes, np.uint8)
        .reshape([2160 * 3 // 2, 3840])
    )
    in_frame = cv2.cvtColor(in_frame, cv2.COLOR_YUV2BGR_NV12)
    cv2.imshow('result', in_frame)
    cv2.waitKey(1)
    video_stream.stdout.flush()

cv2.destroyAllWindows()