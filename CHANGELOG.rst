^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package sentor
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.0.2 (2019-04-12)
------------------
* Merge pull request `#2 <https://github.com/LCAS/sentor/issues/2>`_ from francescodelduchetto/master
  update readme with description of config file usage
* rospy spin instead of 'handmade' spin
* print also the message together with the expression
* Merge branch 'master' into master
* Merge pull request `#1 <https://github.com/LCAS/sentor/issues/1>`_ from francescodelduchetto/2.0
  merge 2.0 to master
* Update README.md
* Update README.md
* Update README.md
* Update README.md
* Contributors: Marc Hanheide, francescodelduchetto

2.0.1 (2019-01-19)
------------------
* Merge pull request `#1 <https://github.com/LCAS/sentor/issues/1>`_ from LCAS/2.0
  Merging 2.0 into master with some modifications for release
* prepare for installation
* prettier prints and longer sleep in loop to avoid None in hz
* added timeout for lambdas and not published
* first commmit version 2.0: yaml file for configuration, singaling also for published, lambda funcs are specified in the yaml as a string
* ehm
* remove logs
* Merge branch 'master' of https://github.com/francescodelduchetto/sentor
* check log to be rem
* another small bit
* remove logs and madd another check to avoid duplicate msg expr in the same list
* some debug logs
* more waiting
* fix bug
* better handling of satsfied expressions as we don't drop anymore expression satisfied very close in time
* Update README.md
* gitignore
* comments and readme
* bug in list inserting elements
* monitoring either the frequency or the expression on msgs
* Merge branch 'master' of github.com:francescodelduchetto/sentor
* tab
* Update README.md
* warning message more significative
* Merge branch 'master' of github.com:francescodelduchetto/sentor
* comment
* elifs instead of ifs
* explanation on usage of filtering
* added possiblity to filter the value of messages and get a warning when it is satisfied
* slightly better printing
* only one warning message when the topic is not published anymore; better terminal printing
* Delete ROSTopicHz.pyc
* Update README.md
* Update README.md
* initial commit
* Contributors: Lindsey User, Marc Hanheide, francescodelduchetto
