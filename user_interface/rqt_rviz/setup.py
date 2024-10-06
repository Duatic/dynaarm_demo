from glob import glob

from setuptools import setup

package_name = "rqt_rviz"

setup(
    name=package_name,
    version="4.5.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/resource", glob("resource/*.*")),
        ("share/" + package_name, ["plugin.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="root",
    maintainer_email="timo@software-byts.de",
    description="Graphical frontend for interacting with the robots.",
    license="tbd",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "rqt_rviz = \
                rqt_rviz.main:main",
        ],
    },
)
