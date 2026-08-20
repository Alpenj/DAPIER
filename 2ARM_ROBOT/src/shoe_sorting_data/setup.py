from setuptools import find_packages, setup


package_name = "shoe_sorting_data"


setup(
    name=package_name,
    version="0.2.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml", "README.md"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Alpenj",
    maintainer_email="29724960+Alpenj@users.noreply.github.com",
    description="DYNA-lite episodes, quality gates, and safe exemplar retrieval for shoe sorting.",
    license="MIT",
    extras_require={"test": ["pytest"]},
    entry_points={
        "console_scripts": [
            "shoe_episode = shoe_sorting_data.cli:main",
        ],
    },
)
