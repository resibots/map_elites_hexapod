# MAP-Elites - Nature version

*Sferes2 experiment* MAP-Elites code for the experiments published in Cully et al. (2015), Nature.

Full reference:
Cully, Antoine, Jeff Clune, Danesh Tarapore, and Jean-Baptiste Mouret. "Robots that can adapt like animals." Nature 521, no. 7553 (2015): 503-507.

- Author-generated [[pdf]](http://www.isir.upmc.fr/files/2015ACLI3468.pdf)
- Nature version (paywall) [[html / pdf]](http://www.nature.com/nature/journal/v521/n7553/full/nature14422.html])

See in particular the supplementary information.

## Authors
- Original author : Antoine Cully
- Other contributions: Jean-Baptiste Mouret

## How to compile

### Dependencies

- [robdyn]: Dynamic simulator
    - Make sure you have installed `ODE` by sources and using this [trick](https://github.com/resibots/robdyn/issues/3#issuecomment-148740769).
    - Get the code: `git clone https://github.com/jbmouret/robdyn.git`
    - Configure for compilation/installation: `./waf configure`
    - Compile with `./waf`
    - Install robdyn on your computer: `sudo ./waf install`
    - For more advanced options, look at [robdyn]'s repo.
- [Sferes2]: A high-performance, multi-core, lightweight, generic C++98 framework for evolutionary computation.
    - As sferes2 is a framework you only need to get the code: `git clone https://github.com/sferes2/sferes2`
    - For more advanced options, look at [Sferes2]'s repo.

### Compiling

- Make sure you have all the dependencies installed/downloaded.
- Go to your `sferes2` root directory
- Create a modules folder (if there's none) and cd to it: `mkdir modules && cd modules`
- Create a symbolic link to robdyn module (or you can copy it): `ln -s your_robdyn_dir/sferes/robdyn`
- Configure for compilation: `./waf configure --robdyn=robdyn_install_dir`
    - You can add other options if you want, like: `--cpp11=yes` to force C++11 compilation
- Compile sferes2 with: `./waf`
- Create an experiment folder (if there's none) and cd to it: `mkdir exp && cd exp`
- Clone map_elites_hexapod: `git clone https://github.com/resibots/map_elites_hexapod.git`
- Go back to your `sfere2` root directory
- Compile the experiment: `./waf --exp map_elites_hexapod`

## How to run it

- Compile the experiment (as shown above)
- Run it (assuming you are on sferes2 root dir):
    - `./build/default/exp/map_elites_hexapod/hexa_duty_text`
    - `./build/debug/exp/map_elites_hexapod/hexa_duty_text` for the debug version

## LICENSE

[CeCILL]

[CeCILL]: http://www.cecill.info/index.en.html
[paper]: http://www.nature.com/nature/journal/v521/n7553/full/nature14422.html
[robdyn]: https://github.com/resibots/robdyn
[Sferes2]: https://github.com/sferes2/sferes2
