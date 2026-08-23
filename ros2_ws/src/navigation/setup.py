from glob import glob

from setuptools import setup


package_name = "navigation"

setup(
    name=package_name,
    version="0.0.0",
    py_modules=["amr_withnav2"],
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        (
            "share/" + package_name + "/maps",
            glob("maps/*.yaml") + glob("maps/*.pgm"),
        ),
        ("share/" + package_name + "/docs", glob("docs/*.md")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="rokey",
    maintainer_email="rokeydg1@gmail.com",
    description="Nav2 configuration and AMR integration for cobot3_project.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "amr_withnav2 = amr_withnav2:main",
        ],
    },
)
