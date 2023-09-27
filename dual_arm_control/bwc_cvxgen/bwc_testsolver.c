/* Produced by CVXGEN, 2019-04-06 15:00:01 -0400.  */
/* CVXGEN is Copyright (C) 2006-2017 Jacob Mattingley, jem@cvxgen.com. */
/* The code in this file is Copyright (C) 2006-2017 Jacob Mattingley. */
/* CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial */
/* applications without prior written permission from Jacob Mattingley. */

/* Filename: bwc_testsolver.c. */
/* Description: Basic test harness for solver.c. */
#include "bwc_solver.h"
bwc_Vars bwc_vars;
bwc_Params bwc_params;
bwc_Workspace bwc_work;
bwc_Settings bwc_settings;
#define NUMTESTS 0
int bwc_main(int argc, char **argv) {
  int num_iters;
#if (NUMTESTS > 0)
  int i;
  double time;
  double time_per;
#endif
  bwc_set_defaults();
  bwc_setup_indexing();
  bwc_load_default_data();
  /* Solve problem instance for the record. */
  bwc_settings.verbose = 1;
  num_iters = bwc_solve();
#ifndef ZERO_LIBRARY_MODE
#if (NUMTESTS > 0)
  /* Now solve multiple problem instances for timing purposes. */
  bwc_settings.verbose = 0;
  bwc_tic();
  for (i = 0; i < NUMTESTS; i++) {
    bwc_solve();
  }
  time = bwc_tocq();
  printf("Timed %d solves over %.3f seconds.\n", NUMTESTS, time);
  time_per = time / NUMTESTS;
  if (time_per > 1) {
    printf("Actual time taken per solve: %.3g s.\n", time_per);
  } else if (time_per > 1e-3) {
    printf("Actual time taken per solve: %.3g ms.\n", 1e3*time_per);
  } else {
    printf("Actual time taken per solve: %.3g us.\n", 1e6*time_per);
  }
#endif
#endif
  return 0;
}
void bwc_load_default_data(void) {
  bwc_params.QFh[0] = 1.101595805149151;
  bwc_params.QFh[1] = 1.4162956452362097;
  bwc_params.QFh[2] = 0.5818094778258887;
  bwc_params.QFh[3] = 1.021655210395326;
  bwc_params.QFh[4] = 1.7858939086953094;
  bwc_params.QFh[5] = 1.7925861778668761;
  bwc_params.QFh[6] = 0.2511706209276725;
  bwc_params.QFh[7] = 0.4144857562763735;
  bwc_params.QFh[8] = 0.10293440660165976;
  bwc_params.QFh[9] = 0.8816196873012729;
  bwc_params.QFh[10] = 0.05975242175713391;
  bwc_params.QFh[11] = 0.9136664487894222;
  bwc_params.pFh[0] = 0.596576190459043;
  bwc_params.pFh[1] = -0.8860508694080989;
  bwc_params.pFh[2] = 0.7050196079205251;
  bwc_params.pFh[3] = 0.3634512696654033;
  bwc_params.pFh[4] = -1.9040724704913385;
  bwc_params.pFh[5] = 0.23541635196352795;
  bwc_params.pFh[6] = -0.9629902123701384;
  bwc_params.pFh[7] = -0.3395952119597214;
  bwc_params.pFh[8] = -0.865899672914725;
  bwc_params.pFh[9] = 0.7725516732519853;
  bwc_params.pFh[10] = -0.23818512931704205;
  bwc_params.pFh[11] = -1.372529046100147;
  bwc_params.Qw[0] = 1.0892980360636895;
  bwc_params.Qw[1] = 1.560629529022734;
  bwc_params.Qw[2] = 0.6127270647523595;
  bwc_params.Qw[3] = 0.4439157678643628;
  bwc_params.Qw[4] = 0.7759425151112975;
  bwc_params.Qw[5] = 1.8727672997208609;
  bwc_params.beta[0] = 1.9519908449458676;
  bwc_params.b1[0] = 0.6895347036512547;
  bwc_params.b1[1] = 1.6113364341535923;
  bwc_params.b1[2] = 1.383003485172717;
  bwc_params.b1[3] = -0.48802383468444344;
  bwc_params.b1[4] = -1.631131964513103;
  bwc_params.b1[5] = 0.6136436100941447;
  bwc_params.Gh_4[0] = 0.2313630495538037;
  bwc_params.Gh_4[1] = -0.5537409477496875;
  bwc_params.Gh_4[2] = -1.0997819806406723;
  bwc_params.Gh_4[3] = -0.3739203344950055;
  bwc_params.Gh_4[4] = -0.12423900520332376;
  bwc_params.Gh_4[5] = -0.923057686995755;
  bwc_params.Gh_4[6] = -0.8328289030982696;
  bwc_params.Gh_4[7] = -0.16925440270808823;
  bwc_params.Gh_4[8] = 1.442135651787706;
  bwc_params.Gh_4[9] = 0.34501161787128565;
  bwc_params.Gh_4[10] = -0.8660485502711608;
  bwc_params.Gh_4[11] = -0.8880899735055947;
  bwc_params.Gh_5[0] = -0.1815116979122129;
  bwc_params.Gh_5[1] = -1.17835862158005;
  bwc_params.Gh_5[2] = -1.1944851558277074;
  bwc_params.Gh_5[3] = 0.05614023926976763;
  bwc_params.Gh_5[4] = -1.6510825248767813;
  bwc_params.Gh_5[5] = -0.06565787059365391;
  bwc_params.Gh_5[6] = -0.5512951504486665;
  bwc_params.Gh_5[7] = 0.8307464872626844;
  bwc_params.Gh_5[8] = 0.9869848924080182;
  bwc_params.Gh_5[9] = 0.7643716874230573;
  bwc_params.Gh_5[10] = 0.7567216550196565;
  bwc_params.Gh_5[11] = -0.5055995034042868;
  bwc_params.Gh_6[0] = 0.6725392189410702;
  bwc_params.Gh_6[1] = -0.6406053441727284;
  bwc_params.Gh_6[2] = 0.29117547947550015;
  bwc_params.Gh_6[3] = -0.6967713677405021;
  bwc_params.Gh_6[4] = -0.21941980294587182;
  bwc_params.Gh_6[5] = -1.753884276680243;
  bwc_params.Gh_6[6] = -1.0292983112626475;
  bwc_params.Gh_6[7] = 1.8864104246942706;
  bwc_params.Gh_6[8] = -1.077663182579704;
  bwc_params.Gh_6[9] = 0.7659100437893209;
  bwc_params.Gh_6[10] = 0.6019074328549583;
  bwc_params.Gh_6[11] = 0.8957565577499285;
  bwc_params.Cplh[0] = -0.09964555746227477;
  bwc_params.Cplh[1] = 0.38665509840745127;
  bwc_params.Cplh[2] = -1.7321223042686946;
  bwc_params.Cplh[3] = -1.7097514487110663;
  bwc_params.Cplh[4] = -1.2040958948116867;
  bwc_params.Cplh[5] = -1.3925560119658358;
  bwc_params.Cprh[0] = -1.5995826216742213;
  bwc_params.Cprh[1] = -1.4828245415645833;
  bwc_params.Cprh[2] = 0.21311092723061398;
  bwc_params.Cprh[3] = -1.248740700304487;
  bwc_params.Cprh[4] = 1.808404972124833;
  bwc_params.Cprh[5] = 0.7264471152297065;
  bwc_params.CLH_1[0] = 0.16407869343908477;
  bwc_params.CLH_1[1] = 0.8287224032315907;
  bwc_params.CLH_1[2] = -0.9444533161899464;
  bwc_params.CLH_1[3] = 1.7069027370149112;
  bwc_params.CLH_1[4] = 1.3567722311998827;
  bwc_params.CLH_1[5] = 0.9052779937121489;
  bwc_params.CLH_2[0] = -0.07904017565835986;
  bwc_params.CLH_2[1] = 1.3684127435065871;
  bwc_params.CLH_2[2] = 0.979009293697437;
  bwc_params.CLH_2[3] = 0.6413036255984501;
  bwc_params.CLH_2[4] = 1.6559010680237511;
  bwc_params.CLH_2[5] = 0.5346622551502991;
  bwc_params.CLH_3[0] = -0.5362376605895625;
  bwc_params.CLH_3[1] = 0.2113782926017822;
  bwc_params.CLH_3[2] = -1.2144776931994525;
  bwc_params.CLH_3[3] = -1.2317108144255875;
  bwc_params.CLH_3[4] = 0.9026784957312834;
  bwc_params.CLH_3[5] = 1.1397468137245244;
  bwc_params.CLH_4[0] = 1.8883934547350631;
  bwc_params.CLH_4[1] = 1.4038856681660068;
  bwc_params.CLH_4[2] = 0.17437730638329096;
  bwc_params.CLH_4[3] = -1.6408365219077408;
  bwc_params.CLH_4[4] = -0.04450702153554875;
  bwc_params.CLH_4[5] = 1.7117453902485025;
  bwc_params.CLH_5[0] = 1.1504727980139053;
  bwc_params.CLH_5[1] = -0.05962309578364744;
  bwc_params.CLH_5[2] = -0.1788825540764547;
  bwc_params.CLH_5[3] = -1.1280569263625857;
  bwc_params.CLH_5[4] = -1.2911464767927057;
  bwc_params.CLH_5[5] = -1.7055053231225696;
  bwc_params.CLH_6[0] = 1.56957275034837;
  bwc_params.CLH_6[1] = 0.5607064675962357;
  bwc_params.CLH_6[2] = -1.4266707301147146;
  bwc_params.CLH_6[3] = -0.3434923211351708;
  bwc_params.CLH_6[4] = -1.8035643024085055;
  bwc_params.CLH_6[5] = -1.1625066019105454;
  bwc_params.CLH_7[0] = 0.9228324965161532;
  bwc_params.CLH_7[1] = 0.6044910817663975;
  bwc_params.CLH_7[2] = -0.0840868104920891;
  bwc_params.CLH_7[3] = -0.900877978017443;
  bwc_params.CLH_7[4] = 0.608892500264739;
  bwc_params.CLH_7[5] = 1.8257980452695217;
  bwc_params.CLH_8[0] = -0.25791777529922877;
  bwc_params.CLH_8[1] = -1.7194699796493191;
  bwc_params.CLH_8[2] = -1.7690740487081298;
  bwc_params.CLH_8[3] = -1.6685159248097703;
  bwc_params.CLH_8[4] = 1.8388287490128845;
  bwc_params.CLH_8[5] = 0.16304334474597537;
  bwc_params.CLH_9[0] = 1.3498497306788897;
  bwc_params.CLH_9[1] = -1.3198658230514613;
  bwc_params.CLH_9[2] = -0.9586197090843394;
  bwc_params.CLH_9[3] = 0.7679100474913709;
  bwc_params.CLH_9[4] = 1.5822813125679343;
  bwc_params.CLH_9[5] = -0.6372460621593619;
  bwc_params.CLH_10[0] = -1.741307208038867;
  bwc_params.CLH_10[1] = 1.456478677642575;
  bwc_params.CLH_10[2] = -0.8365102166820959;
  bwc_params.CLH_10[3] = 0.9643296255982503;
  bwc_params.CLH_10[4] = -1.367865381194024;
  bwc_params.CLH_10[5] = 0.7798537405635035;
  bwc_params.CLH_11[0] = 1.3656784761245926;
  bwc_params.CLH_11[1] = 0.9086083149868371;
  bwc_params.CLH_11[2] = -0.5635699005460344;
  bwc_params.CLH_11[3] = 0.9067590059607915;
  bwc_params.CLH_11[4] = -1.4421315032701587;
  bwc_params.CLH_11[5] = -0.7447235390671119;
  bwc_params.CRH_1[0] = -0.32166897326822186;
  bwc_params.CRH_1[1] = 1.5088481557772684;
  bwc_params.CRH_1[2] = -1.385039165715428;
  bwc_params.CRH_1[3] = 1.5204991609972622;
  bwc_params.CRH_1[4] = 1.1958572768832156;
  bwc_params.CRH_1[5] = 1.8864971883119228;
  bwc_params.CRH_2[0] = -0.5291880667861584;
  bwc_params.CRH_2[1] = -1.1802409243688836;
  bwc_params.CRH_2[2] = -1.037718718661604;
  bwc_params.CRH_2[3] = 1.3114512056856835;
  bwc_params.CRH_2[4] = 1.8609125943756615;
  bwc_params.CRH_2[5] = 0.7952399935216938;
  bwc_params.CRH_3[0] = -0.07001183290468038;
  bwc_params.CRH_3[1] = -0.8518009412754686;
  bwc_params.CRH_3[2] = 1.3347515373726386;
  bwc_params.CRH_3[3] = 1.4887180335977037;
  bwc_params.CRH_3[4] = -1.6314736327976336;
  bwc_params.CRH_3[5] = -1.1362021159208933;
  bwc_params.CRH_4[0] = 1.327044361831466;
  bwc_params.CRH_4[1] = 1.3932155883179842;
  bwc_params.CRH_4[2] = -0.7413880049440107;
  bwc_params.CRH_4[3] = -0.8828216126125747;
  bwc_params.CRH_4[4] = -0.27673991192616;
  bwc_params.CRH_4[5] = 0.15778600105866714;
  bwc_params.CRH_5[0] = -1.6177327399735457;
  bwc_params.CRH_5[1] = 1.3476485548544606;
  bwc_params.CRH_5[2] = 0.13893948140528378;
  bwc_params.CRH_5[3] = 1.0998712601636944;
  bwc_params.CRH_5[4] = -1.0766549376946926;
  bwc_params.CRH_5[5] = 1.8611734044254629;
  bwc_params.CRH_6[0] = 1.0041092292735172;
  bwc_params.CRH_6[1] = -0.6276245424321543;
  bwc_params.CRH_6[2] = 1.794110587839819;
  bwc_params.CRH_6[3] = 0.8020471158650913;
  bwc_params.CRH_6[4] = 1.362244341944948;
  bwc_params.CRH_6[5] = -1.8180107765765245;
  bwc_params.CRH_7[0] = -1.7774338357932473;
  bwc_params.CRH_7[1] = 0.9709490941985153;
  bwc_params.CRH_7[2] = -0.7812542682064318;
  bwc_params.CRH_7[3] = 0.0671374633729811;
  bwc_params.CRH_7[4] = -1.374950305314906;
  bwc_params.CRH_7[5] = 1.9118096386279388;
  bwc_params.CRH_8[0] = 0.011004190697677885;
  bwc_params.CRH_8[1] = 1.3160043138989015;
  bwc_params.CRH_8[2] = -1.7038488148800144;
  bwc_params.CRH_8[3] = -0.08433819112864738;
  bwc_params.CRH_8[4] = -1.7508820783768964;
  bwc_params.CRH_8[5] = 1.536965724350949;
  bwc_params.CRH_9[0] = -0.21675928514816478;
  bwc_params.CRH_9[1] = -1.725800326952653;
  bwc_params.CRH_9[2] = -1.6940148707361717;
  bwc_params.CRH_9[3] = 0.15517063201268;
  bwc_params.CRH_9[4] = -1.697734381979077;
  bwc_params.CRH_9[5] = -1.264910727950229;
  bwc_params.CRH_10[0] = -0.2545716633339441;
  bwc_params.CRH_10[1] = -0.008868675926170244;
  bwc_params.CRH_10[2] = 0.3332476609670296;
  bwc_params.CRH_10[3] = 0.48205072561962936;
  bwc_params.CRH_10[4] = -0.5087540014293261;
  bwc_params.CRH_10[5] = 0.4749463319223195;
  bwc_params.CRH_11[0] = -1.371021366459455;
  bwc_params.CRH_11[1] = -0.8979660982652256;
  bwc_params.CRH_11[2] = 1.194873082385242;
  bwc_params.CRH_11[3] = -1.3876427970939353;
  bwc_params.CRH_11[4] = -1.106708108457053;
  bwc_params.CRH_11[5] = -1.0280872812241797;
}
