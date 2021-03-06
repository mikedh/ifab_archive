MODULE iWeld

	PERS tooldata tool1 := [TRUE,[[0.0578139,0.390291,316.074],[1,0,0,0]],[0.001,[0,0,0.001],[1,0,0,0],0,0,0]];
	!2.06179,-0.0414207,318.517
	PERS seamdata seam1 := [0,0,[0,0,0,0,0,0,0,0,0],0,0,0,0,0,[0,0,0,0,0,0,0,0,0],0,0,[0,0,0,0,0,0,0,0,0],0,0,[0,0,0,0,0,0,0,0,0],0];
	PERS welddata weld1 := [10,0,[3,3,12,123,0,0,23,452,0],[0,0,0,0,0,0,0,0,0]];
	PERS welddata weld2 := [10,0,[2,1,14,105.833,0,0,1,0.42333,0],[0,0,0,0,0,0,0,0,0]];
	
	PROC weld()
		
		! +1"
		MoveJ [[878.16,522.33,804.05],[0.622805,-0.0410897,0.776139,-0.0896333],[0,-2,1,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]], v1000, z50, tool1;
		
		! +.25"
		MoveJ [[889.73,514.89,721.45],[0.622804,-0.0410822,0.77614,-0.0896348],[0,-2,1,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]], v1000, z50, tool1;
		
		! start point (on surface; +0")
		ArcLStart [[894.34,510.20,714.02],[0.635115,-0.0425272,0.766104,-0.0889136],[0,-2,1,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]], v1000, seam1, weld1, fine, tool1;
		
		! point on weld line (on surface) * n-2 points
		ArcL [[894.22,585.20,714.02],[0.635114,-0.0425311,0.766104,-0.0889171],[0,-2,1,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]], v1000, seam1, weld1, z5, tool1;
		
		ArcL [[894.21,585.21,714.00],[0.201935,0.0830554,0.944174,-0.246699],[0,-1,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]], v1000, seam1, weld2, z5, tool1;
		ArcL [[914.88,585.19,713.91],[0.201904,0.0829152,0.944178,-0.246755],[0,-1,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]], v1000, seam1, weld2, z5, tool1;
		ArcL [[914.88,585.19,713.91],[0.201904,0.0829177,0.944178,-0.246752],[0,-1,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]], v1000, seam1, weld2, z5, tool1;
		
		
		! endpoint on weld line (on surface)
		ArcLEnd [[914.88,585.19,713.91],[0.201904,0.0829131,0.944178,-0.246756],[0,-1,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]], v1000, seam1, weld1, fine, tool1;
		
		! +1"
		MoveJ [[914.90,585.20,738.10],[0.201912,0.082921,0.944178,-0.246747],[0,-1,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]], v1000, z50, tool1;
		
	ENDPROC

	PROC zero()
		MoveAbsJ [[0,0,0,0,0,0],[tableHeight,9E+09,9E+09,9E+09,9E+09,9E+09]]\NoEOffs, v1000, fine, tool0;
	ENDPROC

ENDMODULE