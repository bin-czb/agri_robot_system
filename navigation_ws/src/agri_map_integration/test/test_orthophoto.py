from PIL import Image

from agri_map_integration.orthophoto_publisher import (
    build_orthophoto_cloud,
)


def test_transparent_pixels_are_not_published():
    image = Image.new('RGBA', (2, 1))
    image.putdata([(255, 0, 0, 255), (0, 0, 0, 0)])

    cloud = build_orthophoto_cloud(
        image=image,
        frame_id='map',
        resolution=0.05,
        z_offset=-0.05,
    )

    assert cloud.header.frame_id == 'map'
    assert cloud.width == 1
    assert cloud.point_step == 16
    assert len(cloud.data) == 16
