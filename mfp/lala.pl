#join( ' ', map {ucfirst} split( /\./, ( split(/@/, "konsky.kokot\@lala.meh") )[0] ) );
use Data::Dumper; print Dumper
#join ' ', map {ucfirst}
join ' ', map {ucfirst} split /\./, (split /@/, "konsky.kokot\@lala.meh")[0];
#"konsky.kokot\@lala.meh" =~ /([^\.@]+)[^@]/g;

