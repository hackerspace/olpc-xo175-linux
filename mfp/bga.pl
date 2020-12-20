@cols = (undef, qw/A B C D E F G H J K L M N P R T U V W Y AA AB AC AD AE AF AG AH/);
%cols = map { $cols[$_] => $_ } 0..$#cols;

@pins = (
	[ qw/GPIO_0		G	21/ ],
	[ qw/GPIO_1		G	22/ ],
	[ qw/GPIO_2		B	23/ ],
	[ qw/GPIO_3		D	22/ ],
	[ qw/GPIO_4		A	23/ ],
	[ qw/GPIO_5		C	23/ ],
	[ qw/GPIO_6		E	23/ ],
	[ qw/GPIO_7		H	22/ ],
	[ qw/GPIO_8		F	23/ ],
	[ qw/GPIO_9		A	24/ ],
	[ qw/GPIO_10		D	23/ ],
	[ qw/GPIO_11		B	24/ ],
	[ qw/GPIO_12		D	24/ ],
	[ qw/GPIO_13		G	23/ ],
	[ qw/GPIO_14		J	22/ ],
	[ qw/GPIO_15		E	24/ ],
	[ qw/GPIO_16		G	24/ ],
	[ qw/GPIO_17		F	24/ ],
	[ qw/GPIO_18		H	23/ ],
	[ qw/GPIO_19		A	25/ ],
	[ qw/GPIO_20		B	25/ ],
	[ qw/GPIO_21		K	22/ ],
	[ qw/GPIO_22		C	25/ ],
	[ qw/GPIO_23		D	25/ ],
	[ qw/GPIO_24		E	25/ ],
	[ qw/GPIO_25		G	25/ ],
	[ qw/GPIO_26		J	23/ ],
	[ qw/GPIO_27		H	24/ ],
	[ qw/GPIO_28		L	22/ ],
	[ qw/GPIO_29		A	26/ ],
	[ qw/GPIO_30		B	26/ ],
	[ qw/GPIO_31		K	23/ ],
	[ qw/GPIO_32		C	26/ ],
	[ qw/GPIO_33		D	26/ ],
	[ qw/GPIO_34		B	27/ ],
	[ qw/GPIO_35		J	24/ ],
	[ qw/GPIO_36		M	22/ ],
	[ qw/GPIO_37		H	25/ ],
	[ qw/GPIO_38		C	27/ ],
	[ qw/GPIO_39		L	23/ ],
	[ qw/GPIO_40		C	28/ ],
	[ qw/GPIO_41		N	21/ ],
	[ qw/GPIO_42		D	27/ ],
	[ qw/GPIO_43		F	26/ ],
	[ qw/GPIO_44		G	26/ ],
	[ qw/GPIO_45		E	27/ ],
	[ qw/GPIO_46		K	24/ ],
	[ qw/GPIO_47		H	26/ ],
	[ qw/GPIO_48		N	22/ ],
	[ qw/GPIO_49		M	23/ ],
	[ qw/GPIO_50		F	27/ ],
	[ qw/GPIO_51		J	25/ ],
	[ qw/GPIO_52		D	28/ ],
	[ qw/GPIO_53		E	28/ ],
	[ qw/GPIO_54		L	24/ ],
	[ qw/GPIO_55		F	28/ ],
	[ qw/GPIO_56		G	27/ ],
	[ qw/GPIO_57		P	22/ ],
	[ qw/GPIO_58		G	28/ ],
	[ qw/GPIO_59		AD	13/ ],
	[ qw/GPIO_60		AE	13/ ],
	[ qw/GPIO_61		AF	13/ ],
	[ qw/GPIO_62		AH	13/ ],
	[ qw/GPIO_63		AG	13/ ],
	[ qw/GPIO_64		AC	12/ ],
	[ qw/GPIO_65		AD	12/ ],
	[ qw/GPIO_66		AE	12/ ],
	[ qw/GPIO_67		AF	12/ ],
	[ qw/GPIO_68		AG	12/ ],
	[ qw/GPIO_69		AH	12/ ],
	[ qw/GPIO_70		AC	11/ ],
	[ qw/GPIO_71		AD	11/ ],
	[ qw/GPIO_72		AG	11/ ],
	[ qw/GPIO_73		AE	11/ ],
	[ qw/GPIO_74		W	23/ ],
	[ qw/GPIO_75		V	25/ ],
	[ qw/GPIO_76		W	22/ ],
	[ qw/GPIO_77		Y	25/ ],
	[ qw/GPIO_78		W	24/ ],
	[ qw/GPIO_79		Y	22/ ],
	[ qw/GPIO_80		Y	23/ ],
	[ qw/GPIO_81		Y	24/ ],
	[ qw/GPIO_82		AA	20/ ],
	[ qw/GPIO_83		AA	24/ ],
	[ qw/GPIO_84		AA	23/ ],
	[ qw/GPIO_85		AB	21/ ],
	[ qw/GPIO_86		AB	24/ ],
	[ qw/GPIO_87		AA	25/ ],
	[ qw/GPIO_88		AB	22/ ],
	[ qw/GPIO_89		AB	25/ ],
	[ qw/GPIO_90		AB	23/ ],
	[ qw/GPIO_91		AB	27/ ],
	[ qw/GPIO_92		AB	28/ ],
	[ qw/GPIO_93		AB	20/ ],
	[ qw/GPIO_94		AC	24/ ],
	[ qw/GPIO_95		AC	21/ ],
	[ qw/GPIO_96		AC	23/ ],
	[ qw/GPIO_97		AA	19/ ],
	[ qw/GPIO_98		EC	25/ ],
	[ qw/GPIO_99		AC	27/ ],
	[ qw/GPIO_100	AC	26/ ],
	[ qw/GPIO_101	AB	19/ ],
	[ qw/GPIO_102	D	19/ ],
	[ qw/GPIO_103	C	19/ ],
	[ qw/GPIO_104	AE	25/ ],
	[ qw/GPIO_105	AE	22/ ],
	[ qw/GPIO_106	AF	27/ ],
	[ qw/GPIO_107	AE	21/ ],
	[ qw/GPIO_108	AD	23/ ],
	[ qw/GPIO_109	AD	24/ ],
	[ qw/GPIO_110	AD	25/ ],
	[ qw/GPIO_111	AE	26/ ],
	[ qw/GPIO_112	AH	25/ ],
	[ qw/GPIO_113	AC	17/ ],
	[ qw/GPIO_114	U	21/ ],
	[ qw/GPIO_115	N	23/ ],
	[ qw/GPIO_116	H	27/ ],
	[ qw/GPIO_117	H	28/ ],
	[ qw/GPIO_118	J	28/ ],
	[ qw/GPIO_119	M	24/ ],
	[ qw/GPIO_120	J	27/ ],
	[ qw/GPIO_121	K	26/ ],
	[ qw/GPIO_122	L	25/ ],
	[ qw/GPIO_123	R	23/ ],
	[ qw/GPIO_124	A	20/ ],
	[ qw/GPIO_125	E	20/ ],
	[ qw/GPIO_126	AF	11/ ],
	[ qw/GPIO_127	AE	10/ ],
	[ qw/GPIO_128	AH	11/ ],
	[ qw/GPIO_129	AF	10/ ],
	[ qw/GPIO_130	AD	10/ ],
	[ qw/GPIO_131	D	20/ ],
	[ qw/GPIO_132	B	20/ ],
	[ qw/GPIO_133	A	21/ ],
	[ qw/GPIO_134	B	21/ ],
	[ qw/GPIO_135	F	20/ ],
	[ qw/GPIO_136	C	21/ ],
	[ qw/GPIO_137	D	21/ ],
	[ qw/GPIO_138	B	22/ ],
	[ qw/GPIO_139	AC	10/ ],
	[ qw/GPIO_140	A	22/ ],
	[ qw/GPIO_141	C	22/ ],
	[ qw/GPIO_142	B	19/ ],
	[ qw/GPIO_143	AF	23/ ],
	[ qw/GPIO_144	AF	24/ ],
	[ qw/GPIO_145	AC	19/ ],
	[ qw/GPIO_146	AC	20/ ],
	[ qw/GPIO_147	AG	25/ ],
	[ qw/GPIO_148	AG	26/ ],
	[ qw/GPIO_149	AA	16/ ],
	[ qw/GPIO_150	AH	26/ ],
	[ qw/GPIO_151	AG	24/ ],
	[ qw/GPIO_152	AC	18/ ],
	[ qw/GPIO_153	AD	18/ ],
	[ qw/GPIO_154	AB	17/ ],
	[ qw/GPIO_155	AE	19/ ],
	[ qw/GPIO_156	L	28/ ],
	[ qw/GPIO_157	N	25/ ],
	[ qw/GPIO_158	M	27/ ],
	[ qw/GPIO_159	N	26/ ],
	[ qw/GPIO_160	AH	24/ ],
	[ qw/GPIO_161	AD	26/ ],
	[ qw/GPIO_162	AD	24/ ],
	[ qw/GPIO_163	AD	28/ ],
	[ qw/GPIO_164	AD	25/ ],
	[ qw/GPIO_165	AE	23/ ],
	[ qw/GPIO_166	AF	26/ ],
	[ qw/GPIO_167	AF	25/ ],
	[ qw/GPIO_168	AF	28/ ],
	[ qw/TWSI4_SCL	D	18/ ],
	[ qw/TWSI4_SDA	A	19/ ],
	[ qw/GPIO_171	L	27/ ],
	[ qw/VCXO_OUT	N	27/ ],
	[ qw/VCXO_REQ	T	24/ ],
);

use Data::Dumper;
#die Dumper \%cols;

for (@pins) {
	($pin, $col, $row) = @$_;
	($c, $r) = ($cols{$col}, $row);
	#warn Dumper [ $pin, $c, $r ];
	$array->[$r][$c] = $pin;
}

print <<END;
<html>
<head>
<style>
td { font-size: 6pt; }
td { width: 1em; }
td { height: 3em; }
</style>
</head>
<body>
<table border=1>
END
for $row (0..28) {
	print "<tr>";
	for $col (0..28) {
		printf "<td>%s</td>", $array->[$row][$col];
		#warn $row;
	}
	print "</tr>\n";
}
#warn Dumper $array;
print <<END;
</table>
</body>
</head>
END

__END__

CLK_REQ			P25 -- 171?

19X19m package
0.65mm ball pitch

28*28


AH
AG
AF
AE
AD
AC
AB
AA
Y
W
V
U
T
R
P
N
M
L
K
J
H
G
F
E
D
C
B
A -
  1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24 25 26 27 28 
