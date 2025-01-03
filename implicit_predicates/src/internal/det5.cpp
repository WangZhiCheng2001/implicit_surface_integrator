#include <implicit_point.h>

#pragma intrinsic(fabs)

int det5_filtered(double a_,
                  double b_,
                  double c_,
                  double d_,
                  double e_,
                  double f_,
                  double g_,
                  double h_,
                  double i_,
                  double j_,
                  double k_,
                  double l_,
                  double m_,
                  double n_,
                  double o_,
                  double p_,
                  double q_,
                  double r_,
                  double s_,
                  double t_,
                  double u_,
                  double v_,
                  double w_,
                  double x_,
                  double y_)
{
    double ag       = a_ * g_;
    double ah       = a_ * h_;
    double ai       = a_ * i_;
    double aj       = a_ * j_;
    double bf       = b_ * f_;
    double bh       = b_ * h_;
    double bi       = b_ * i_;
    double bj       = b_ * j_;
    double cf       = c_ * f_;
    double cg       = c_ * g_;
    double ci       = c_ * i_;
    double cj       = c_ * j_;
    double df       = d_ * f_;
    double dg       = d_ * g_;
    double dh       = d_ * h_;
    double dj       = d_ * j_;
    double ef       = e_ * f_;
    double eg       = e_ * g_;
    double eh       = e_ * h_;
    double ei       = e_ * i_;
    double pv       = p_ * v_;
    double pw       = p_ * w_;
    double px       = p_ * x_;
    double py       = p_ * y_;
    double qu       = q_ * u_;
    double qw       = q_ * w_;
    double qx       = q_ * x_;
    double qy       = q_ * y_;
    double ru       = r_ * u_;
    double rv       = r_ * v_;
    double rx       = r_ * x_;
    double ry       = r_ * y_;
    double su       = s_ * u_;
    double sv       = s_ * v_;
    double sw       = s_ * w_;
    double sy       = s_ * y_;
    double tu       = t_ * u_;
    double tv       = t_ * v_;
    double tw       = t_ * w_;
    double tx       = t_ * x_;
    double pv_qu    = pv - qu;
    double pw_ru    = pw - ru;
    double px_su    = px - su;
    double py_tu    = py - tu;
    double qw_rv    = qx - rv;
    double qx_sv    = qx - sv;
    double qy_tv    = qy - tv;
    double rx_sw    = rx - sw;
    double ry_tw    = ry - tw;
    double sy_tx    = sy - tx;
    double k__qw_rv = k_ * qw_rv;
    double k__qx_sv = k_ * qx_sv;
    double k__qy_tv = k_ * qy_tv;
    double k__rx_sw = k_ * rx_sw;
    double k__ry_tw = k_ * ry_tw;
    double k__sy_tx = k_ * sy_tx;
    double l__pw_ru = l_ * pw_ru;
    double l__px_su = l_ * px_su;
    double l__py_tu = l_ * py_tu;
    double l__rx_sw = l_ * rx_sw;
    double l__ry_tw = l_ * ry_tw;
    double l__sy_tx = l_ * sy_tx;
    double m__pv_qu = m_ * pv_qu;
    double m__px_su = m_ * px_su;
    double m__py_tu = m_ * py_tu;
    double m__qx_sv = m_ * qx_sv;
    double m__qy_tv = m_ * qy_tv;
    double m__sy_tx = m_ * sy_tx;
    double n__pv_qu = n_ * pv_qu;
    double n__pw_ru = n_ * pw_ru;
    double n__py_tu = n_ * py_tu;
    double n__qy_tv = n_ * qy_tv;
    double n__qw_rv = n_ * qw_rv;
    double n__ry_tw = n_ * ry_tw;
    double o__pv_qu = o_ * pv_qu;
    double o__pw_ru = o_ * pw_ru;
    double o__px_su = o_ * px_su;
    double o__qw_rv = o_ * qw_rv;
    double o__qx_sv = o_ * qx_sv;
    double o__rx_sw = o_ * rx_sw;
    double term1_1  = m__sy_tx - n__ry_tw;
    double term1_2  = term1_1 + o__rx_sw;
    double term1    = ag * term1_2;
    double term2_1  = l__sy_tx - n__qy_tv;
    double term2_2  = term2_1 + o__qx_sv;
    double term2    = ah * term2_2;
    double term3_1  = l__ry_tw - m__qy_tv;
    double term3_2  = term3_1 + o__qw_rv;
    double term3    = ai * term3_2;
    double term4_1  = l__rx_sw - m__qx_sv;
    double term4_2  = term4_1 + n__qw_rv;
    double term4    = aj * term4_2;
    double term5_1  = m__sy_tx - n__ry_tw;
    double term5_2  = term5_1 + o__rx_sw;
    double term5    = bf * term5_2;
    double term6_1  = k__sy_tx - n__py_tu;
    double term6_2  = term6_1 + o__px_su;
    double term6    = bh * term6_2;
    double term7_1  = k__ry_tw - m__py_tu;
    double term7_2  = term7_1 + o__pw_ru;
    double term7    = bi * term7_2;
    double term8_1  = k__rx_sw - m__px_su;
    double term8_2  = term8_1 + n__pw_ru;
    double term8    = bj * term8_2;
    double term9_1  = l__sy_tx - n__qy_tv;
    double term9_2  = term9_1 + o__qx_sv;
    double term9    = cf * term9_2;
    double term10_1 = k__sy_tx - n__py_tu;
    double term10_2 = term10_1 + o__px_su;
    double term10   = cg * term10_2;
    double term11_1 = k__qy_tv - l__py_tu;
    double term11_2 = term11_1 + o__pv_qu;
    double term11   = ci * term11_2;
    double term12_1 = k__qx_sv - l__px_su;
    double term12_2 = term12_1 + n__pv_qu;
    double term12   = cj * term12_2;
    double term13_1 = l__ry_tw - m__qy_tv;
    double term13_2 = term13_1 + o__qw_rv;
    double term13   = df * term13_2;
    double term14_1 = k__ry_tw - m__py_tu;
    double term14_2 = term14_1 + o__pw_ru;
    double term14   = dg * term14_2;
    double term15_1 = k__qy_tv - l__py_tu;
    double term15_2 = term15_1 + o__pv_qu;
    double term15   = dh * term15_2;
    double term16_1 = k__qw_rv - l__pw_ru;
    double term16_2 = term16_1 + m__pv_qu;
    double term16   = dj * term16_2;
    double term17_1 = l__rx_sw - m__qx_sv;
    double term17_2 = term17_1 + n__qw_rv;
    double term17   = ef * term17_2;
    double term18_1 = k__rx_sw - m__px_su;
    double term18_2 = term18_1 + n__pw_ru;
    double term18   = eg * term18_2;
    double term19_1 = k__qx_sv - l__px_su;
    double term19_2 = term19_1 + n__pv_qu;
    double term19   = eh * term19_2;
    double term20_1 = k__qw_rv - l__pw_ru;
    double term20_2 = term20_1 + m__pv_qu;
    double term20   = ei * term20_2;
    double result1  = term1 - term2;
    double result2  = result1 + term3;
    double result3  = result2 - term4;
    double result4  = result3 - term5;
    double result5  = result4 + term6;
    double result6  = result5 - term7;
    double result7  = result6 + term8;
    double result8  = result7 + term9;
    double result9  = result8 - term10;
    double result10 = result9 + term11;
    double result11 = result10 - term12;
    double result12 = result11 - term13;
    double result13 = result12 + term14;
    double result14 = result13 - term15;
    double result15 = result14 + term16;
    double result16 = result15 + term17;
    double result17 = result16 - term18;
    double result18 = result17 + term19;
    double result   = result18 - term20;

    double _tmp_fabs;

    double max_var = 0.0;
    if ((_tmp_fabs = fabs(a_)) > max_var) max_var = _tmp_fabs;
    if ((_tmp_fabs = fabs(b_)) > max_var) max_var = _tmp_fabs;
    if ((_tmp_fabs = fabs(c_)) > max_var) max_var = _tmp_fabs;
    if ((_tmp_fabs = fabs(d_)) > max_var) max_var = _tmp_fabs;
    if ((_tmp_fabs = fabs(e_)) > max_var) max_var = _tmp_fabs;
    if ((_tmp_fabs = fabs(f_)) > max_var) max_var = _tmp_fabs;
    if ((_tmp_fabs = fabs(g_)) > max_var) max_var = _tmp_fabs;
    if ((_tmp_fabs = fabs(h_)) > max_var) max_var = _tmp_fabs;
    if ((_tmp_fabs = fabs(i_)) > max_var) max_var = _tmp_fabs;
    if ((_tmp_fabs = fabs(j_)) > max_var) max_var = _tmp_fabs;
    if ((_tmp_fabs = fabs(k_)) > max_var) max_var = _tmp_fabs;
    if ((_tmp_fabs = fabs(l_)) > max_var) max_var = _tmp_fabs;
    if ((_tmp_fabs = fabs(m_)) > max_var) max_var = _tmp_fabs;
    if ((_tmp_fabs = fabs(n_)) > max_var) max_var = _tmp_fabs;
    if ((_tmp_fabs = fabs(o_)) > max_var) max_var = _tmp_fabs;
    if ((_tmp_fabs = fabs(p_)) > max_var) max_var = _tmp_fabs;
    if ((_tmp_fabs = fabs(q_)) > max_var) max_var = _tmp_fabs;
    if ((_tmp_fabs = fabs(r_)) > max_var) max_var = _tmp_fabs;
    if ((_tmp_fabs = fabs(s_)) > max_var) max_var = _tmp_fabs;
    if ((_tmp_fabs = fabs(t_)) > max_var) max_var = _tmp_fabs;
    if ((_tmp_fabs = fabs(u_)) > max_var) max_var = _tmp_fabs;
    if ((_tmp_fabs = fabs(v_)) > max_var) max_var = _tmp_fabs;
    if ((_tmp_fabs = fabs(w_)) > max_var) max_var = _tmp_fabs;
    if ((_tmp_fabs = fabs(x_)) > max_var) max_var = _tmp_fabs;
    if ((_tmp_fabs = fabs(y_)) > max_var) max_var = _tmp_fabs;
    double epsilon  = max_var;
    epsilon        *= epsilon;
    epsilon        *= epsilon;
    epsilon        *= max_var;
    epsilon        *= 1.74971148680925e-13;
    if (result > epsilon) return IP_Sign::POSITIVE;
    if (-result > epsilon) return IP_Sign::NEGATIVE;
    return Filtered_Sign::UNCERTAIN;
}

int det5_interval(interval_number a_,
                  interval_number b_,
                  interval_number c_,
                  interval_number d_,
                  interval_number e_,
                  interval_number f_,
                  interval_number g_,
                  interval_number h_,
                  interval_number i_,
                  interval_number j_,
                  interval_number k_,
                  interval_number l_,
                  interval_number m_,
                  interval_number n_,
                  interval_number o_,
                  interval_number p_,
                  interval_number q_,
                  interval_number r_,
                  interval_number s_,
                  interval_number t_,
                  interval_number u_,
                  interval_number v_,
                  interval_number w_,
                  interval_number x_,
                  interval_number y_)
{
    setFPUModeToRoundUP();
    interval_number ag(a_ * g_);
    interval_number ah(a_ * h_);
    interval_number ai(a_ * i_);
    interval_number aj(a_ * j_);
    interval_number bf(b_ * f_);
    interval_number bh(b_ * h_);
    interval_number bi(b_ * i_);
    interval_number bj(b_ * j_);
    interval_number cf(c_ * f_);
    interval_number cg(c_ * g_);
    interval_number ci(c_ * i_);
    interval_number cj(c_ * j_);
    interval_number df(d_ * f_);
    interval_number dg(d_ * g_);
    interval_number dh(d_ * h_);
    interval_number dj(d_ * j_);
    interval_number ef(e_ * f_);
    interval_number eg(e_ * g_);
    interval_number eh(e_ * h_);
    interval_number ei(e_ * i_);
    interval_number pv(p_ * v_);
    interval_number pw(p_ * w_);
    interval_number px(p_ * x_);
    interval_number py(p_ * y_);
    interval_number qu(q_ * u_);
    interval_number qw(q_ * w_);
    interval_number qx(q_ * x_);
    interval_number qy(q_ * y_);
    interval_number ru(r_ * u_);
    interval_number rv(r_ * v_);
    interval_number rx(r_ * x_);
    interval_number ry(r_ * y_);
    interval_number su(s_ * u_);
    interval_number sv(s_ * v_);
    interval_number sw(s_ * w_);
    interval_number sy(s_ * y_);
    interval_number tu(t_ * u_);
    interval_number tv(t_ * v_);
    interval_number tw(t_ * w_);
    interval_number tx(t_ * x_);
    interval_number pv_qu(pv - qu);
    interval_number pw_ru(pw - ru);
    interval_number px_su(px - su);
    interval_number py_tu(py - tu);
    interval_number qw_rv(qx - rv);
    interval_number qx_sv(qx - sv);
    interval_number qy_tv(qy - tv);
    interval_number rx_sw(rx - sw);
    interval_number ry_tw(ry - tw);
    interval_number sy_tx(sy - tx);
    interval_number k__qw_rv(k_ * qw_rv);
    interval_number k__qx_sv(k_ * qx_sv);
    interval_number k__qy_tv(k_ * qy_tv);
    interval_number k__rx_sw(k_ * rx_sw);
    interval_number k__ry_tw(k_ * ry_tw);
    interval_number k__sy_tx(k_ * sy_tx);
    interval_number l__pw_ru(l_ * pw_ru);
    interval_number l__px_su(l_ * px_su);
    interval_number l__py_tu(l_ * py_tu);
    interval_number l__rx_sw(l_ * rx_sw);
    interval_number l__ry_tw(l_ * ry_tw);
    interval_number l__sy_tx(l_ * sy_tx);
    interval_number m__pv_qu(m_ * pv_qu);
    interval_number m__px_su(m_ * px_su);
    interval_number m__py_tu(m_ * py_tu);
    interval_number m__qx_sv(m_ * qx_sv);
    interval_number m__qy_tv(m_ * qy_tv);
    interval_number m__sy_tx(m_ * sy_tx);
    interval_number n__pv_qu(n_ * pv_qu);
    interval_number n__pw_ru(n_ * pw_ru);
    interval_number n__py_tu(n_ * py_tu);
    interval_number n__qy_tv(n_ * qy_tv);
    interval_number n__qw_rv(n_ * qw_rv);
    interval_number n__ry_tw(n_ * ry_tw);
    interval_number o__pv_qu(o_ * pv_qu);
    interval_number o__pw_ru(o_ * pw_ru);
    interval_number o__px_su(o_ * px_su);
    interval_number o__qw_rv(o_ * qw_rv);
    interval_number o__qx_sv(o_ * qx_sv);
    interval_number o__rx_sw(o_ * rx_sw);
    interval_number term1_1(m__sy_tx - n__ry_tw);
    interval_number term1_2(term1_1 + o__rx_sw);
    interval_number term1(ag * term1_2);
    interval_number term2_1(l__sy_tx - n__qy_tv);
    interval_number term2_2(term2_1 + o__qx_sv);
    interval_number term2(ah * term2_2);
    interval_number term3_1(l__ry_tw - m__qy_tv);
    interval_number term3_2(term3_1 + o__qw_rv);
    interval_number term3(ai * term3_2);
    interval_number term4_1(l__rx_sw - m__qx_sv);
    interval_number term4_2(term4_1 + n__qw_rv);
    interval_number term4(aj * term4_2);
    interval_number term5_1(m__sy_tx - n__ry_tw);
    interval_number term5_2(term5_1 + o__rx_sw);
    interval_number term5(bf * term5_2);
    interval_number term6_1(k__sy_tx - n__py_tu);
    interval_number term6_2(term6_1 + o__px_su);
    interval_number term6(bh * term6_2);
    interval_number term7_1(k__ry_tw - m__py_tu);
    interval_number term7_2(term7_1 + o__pw_ru);
    interval_number term7(bi * term7_2);
    interval_number term8_1(k__rx_sw - m__px_su);
    interval_number term8_2(term8_1 + n__pw_ru);
    interval_number term8(bj * term8_2);
    interval_number term9_1(l__sy_tx - n__qy_tv);
    interval_number term9_2(term9_1 + o__qx_sv);
    interval_number term9(cf * term9_2);
    interval_number term10_1(k__sy_tx - n__py_tu);
    interval_number term10_2(term10_1 + o__px_su);
    interval_number term10(cg * term10_2);
    interval_number term11_1(k__qy_tv - l__py_tu);
    interval_number term11_2(term11_1 + o__pv_qu);
    interval_number term11(ci * term11_2);
    interval_number term12_1(k__qx_sv - l__px_su);
    interval_number term12_2(term12_1 + n__pv_qu);
    interval_number term12(cj * term12_2);
    interval_number term13_1(l__ry_tw - m__qy_tv);
    interval_number term13_2(term13_1 + o__qw_rv);
    interval_number term13(df * term13_2);
    interval_number term14_1(k__ry_tw - m__py_tu);
    interval_number term14_2(term14_1 + o__pw_ru);
    interval_number term14(dg * term14_2);
    interval_number term15_1(k__qy_tv - l__py_tu);
    interval_number term15_2(term15_1 + o__pv_qu);
    interval_number term15(dh * term15_2);
    interval_number term16_1(k__qw_rv - l__pw_ru);
    interval_number term16_2(term16_1 + m__pv_qu);
    interval_number term16(dj * term16_2);
    interval_number term17_1(l__rx_sw - m__qx_sv);
    interval_number term17_2(term17_1 + n__qw_rv);
    interval_number term17(ef * term17_2);
    interval_number term18_1(k__rx_sw - m__px_su);
    interval_number term18_2(term18_1 + n__pw_ru);
    interval_number term18(eg * term18_2);
    interval_number term19_1(k__qx_sv - l__px_su);
    interval_number term19_2(term19_1 + n__pv_qu);
    interval_number term19(eh * term19_2);
    interval_number term20_1(k__qw_rv - l__pw_ru);
    interval_number term20_2(term20_1 + m__pv_qu);
    interval_number term20(ei * term20_2);
    interval_number result1(term1 - term2);
    interval_number result2(result1 + term3);
    interval_number result3(result2 - term4);
    interval_number result4(result3 - term5);
    interval_number result5(result4 + term6);
    interval_number result6(result5 - term7);
    interval_number result7(result6 + term8);
    interval_number result8(result7 + term9);
    interval_number result9(result8 - term10);
    interval_number result10(result9 + term11);
    interval_number result11(result10 - term12);
    interval_number result12(result11 - term13);
    interval_number result13(result12 + term14);
    interval_number result14(result13 - term15);
    interval_number result15(result14 + term16);
    interval_number result16(result15 + term17);
    interval_number result17(result16 - term18);
    interval_number result18(result17 + term19);
    interval_number result(result18 - term20);
    setFPUModeToRoundNEAR();

    if (!result.signIsReliable()) return Filtered_Sign::UNCERTAIN;
    return result.sign();
}

int det5_exact(double a_,
               double b_,
               double c_,
               double d_,
               double e_,
               double f_,
               double g_,
               double h_,
               double i_,
               double j_,
               double k_,
               double l_,
               double m_,
               double n_,
               double o_,
               double p_,
               double q_,
               double r_,
               double s_,
               double t_,
               double u_,
               double v_,
               double w_,
               double x_,
               double y_)
{
    expansionObject o;
    double          ag[2];
    o.Two_Prod(a_, g_, ag);
    double ah[2];
    o.Two_Prod(a_, h_, ah);
    double ai[2];
    o.Two_Prod(a_, i_, ai);
    double aj[2];
    o.Two_Prod(a_, j_, aj);
    double bf[2];
    o.Two_Prod(b_, f_, bf);
    double bh[2];
    o.Two_Prod(b_, h_, bh);
    double bi[2];
    o.Two_Prod(b_, i_, bi);
    double bj[2];
    o.Two_Prod(b_, j_, bj);
    double cf[2];
    o.Two_Prod(c_, f_, cf);
    double cg[2];
    o.Two_Prod(c_, g_, cg);
    double ci[2];
    o.Two_Prod(c_, i_, ci);
    double cj[2];
    o.Two_Prod(c_, j_, cj);
    double df[2];
    o.Two_Prod(d_, f_, df);
    double dg[2];
    o.Two_Prod(d_, g_, dg);
    double dh[2];
    o.Two_Prod(d_, h_, dh);
    double dj[2];
    o.Two_Prod(d_, j_, dj);
    double ef[2];
    o.Two_Prod(e_, f_, ef);
    double eg[2];
    o.Two_Prod(e_, g_, eg);
    double eh[2];
    o.Two_Prod(e_, h_, eh);
    double ei[2];
    o.Two_Prod(e_, i_, ei);
    double pv[2];
    o.Two_Prod(p_, v_, pv);
    double pw[2];
    o.Two_Prod(p_, w_, pw);
    double px[2];
    o.Two_Prod(p_, x_, px);
    double py[2];
    o.Two_Prod(p_, y_, py);
    double qu[2];
    o.Two_Prod(q_, u_, qu);
    double qw[2];
    o.Two_Prod(q_, w_, qw);
    double qx[2];
    o.Two_Prod(q_, x_, qx);
    double qy[2];
    o.Two_Prod(q_, y_, qy);
    double ru[2];
    o.Two_Prod(r_, u_, ru);
    double rv[2];
    o.Two_Prod(r_, v_, rv);
    double rx[2];
    o.Two_Prod(r_, x_, rx);
    double ry[2];
    o.Two_Prod(r_, y_, ry);
    double su[2];
    o.Two_Prod(s_, u_, su);
    double sv[2];
    o.Two_Prod(s_, v_, sv);
    double sw[2];
    o.Two_Prod(s_, w_, sw);
    double sy[2];
    o.Two_Prod(s_, y_, sy);
    double tu[2];
    o.Two_Prod(t_, u_, tu);
    double tv[2];
    o.Two_Prod(t_, v_, tv);
    double tw[2];
    o.Two_Prod(t_, w_, tw);
    double tx[2];
    o.Two_Prod(t_, x_, tx);
    double pv_qu[4];
    o.Two_Two_Diff(pv, qu, pv_qu);
    double pw_ru[4];
    o.Two_Two_Diff(pw, ru, pw_ru);
    double px_su[4];
    o.Two_Two_Diff(px, su, px_su);
    double py_tu[4];
    o.Two_Two_Diff(py, tu, py_tu);
    double qw_rv[4];
    o.Two_Two_Diff(qx, rv, qw_rv);
    double qx_sv[4];
    o.Two_Two_Diff(qx, sv, qx_sv);
    double qy_tv[4];
    o.Two_Two_Diff(qy, tv, qy_tv);
    double rx_sw[4];
    o.Two_Two_Diff(rx, sw, rx_sw);
    double ry_tw[4];
    o.Two_Two_Diff(ry, tw, ry_tw);
    double sy_tx[4];
    o.Two_Two_Diff(sy, tx, sy_tx);
    double k__qw_rv[8];
    int    k__qw_rv_len = o.Gen_Scale(4, qw_rv, k_, k__qw_rv);
    double k__qx_sv[8];
    int    k__qx_sv_len = o.Gen_Scale(4, qx_sv, k_, k__qx_sv);
    double k__qy_tv[8];
    int    k__qy_tv_len = o.Gen_Scale(4, qy_tv, k_, k__qy_tv);
    double k__rx_sw[8];
    int    k__rx_sw_len = o.Gen_Scale(4, rx_sw, k_, k__rx_sw);
    double k__ry_tw[8];
    int    k__ry_tw_len = o.Gen_Scale(4, ry_tw, k_, k__ry_tw);
    double k__sy_tx[8];
    int    k__sy_tx_len = o.Gen_Scale(4, sy_tx, k_, k__sy_tx);
    double l__pw_ru[8];
    int    l__pw_ru_len = o.Gen_Scale(4, pw_ru, l_, l__pw_ru);
    double l__px_su[8];
    int    l__px_su_len = o.Gen_Scale(4, px_su, l_, l__px_su);
    double l__py_tu[8];
    int    l__py_tu_len = o.Gen_Scale(4, py_tu, l_, l__py_tu);
    double l__rx_sw[8];
    int    l__rx_sw_len = o.Gen_Scale(4, rx_sw, l_, l__rx_sw);
    double l__ry_tw[8];
    int    l__ry_tw_len = o.Gen_Scale(4, ry_tw, l_, l__ry_tw);
    double l__sy_tx[8];
    int    l__sy_tx_len = o.Gen_Scale(4, sy_tx, l_, l__sy_tx);
    double m__pv_qu[8];
    int    m__pv_qu_len = o.Gen_Scale(4, pv_qu, m_, m__pv_qu);
    double m__px_su[8];
    int    m__px_su_len = o.Gen_Scale(4, px_su, m_, m__px_su);
    double m__py_tu[8];
    int    m__py_tu_len = o.Gen_Scale(4, py_tu, m_, m__py_tu);
    double m__qx_sv[8];
    int    m__qx_sv_len = o.Gen_Scale(4, qx_sv, m_, m__qx_sv);
    double m__qy_tv[8];
    int    m__qy_tv_len = o.Gen_Scale(4, qy_tv, m_, m__qy_tv);
    double m__sy_tx[8];
    int    m__sy_tx_len = o.Gen_Scale(4, sy_tx, m_, m__sy_tx);
    double n__pv_qu[8];
    int    n__pv_qu_len = o.Gen_Scale(4, pv_qu, n_, n__pv_qu);
    double n__pw_ru[8];
    int    n__pw_ru_len = o.Gen_Scale(4, pw_ru, n_, n__pw_ru);
    double n__py_tu[8];
    int    n__py_tu_len = o.Gen_Scale(4, py_tu, n_, n__py_tu);
    double n__qy_tv[8];
    int    n__qy_tv_len = o.Gen_Scale(4, qy_tv, n_, n__qy_tv);
    double n__qw_rv[8];
    int    n__qw_rv_len = o.Gen_Scale(4, qw_rv, n_, n__qw_rv);
    double n__ry_tw[8];
    int    n__ry_tw_len = o.Gen_Scale(4, ry_tw, n_, n__ry_tw);
    double o__pv_qu[8];
    int    o__pv_qu_len = o.Gen_Scale(4, pv_qu, o_, o__pv_qu);
    double o__pw_ru[8];
    int    o__pw_ru_len = o.Gen_Scale(4, pw_ru, o_, o__pw_ru);
    double o__px_su[8];
    int    o__px_su_len = o.Gen_Scale(4, px_su, o_, o__px_su);
    double o__qw_rv[8];
    int    o__qw_rv_len = o.Gen_Scale(4, qw_rv, o_, o__qw_rv);
    double o__qx_sv[8];
    int    o__qx_sv_len = o.Gen_Scale(4, qx_sv, o_, o__qx_sv);
    double o__rx_sw[8];
    int    o__rx_sw_len = o.Gen_Scale(4, rx_sw, o_, o__rx_sw);
    double term1_1[16];
    int    term1_1_len = o.Gen_Diff(m__sy_tx_len, m__sy_tx, n__ry_tw_len, n__ry_tw, term1_1);
    double term1_2_p[16], *term1_2 = term1_2_p;
    int    term1_2_len = o.Gen_Sum_With_PreAlloc(term1_1_len, term1_1, o__rx_sw_len, o__rx_sw, &term1_2, 16);
    double term1_p[16], *term1 = term1_p;
    int    term1_len = o.Gen_Product_With_PreAlloc(2, ag, term1_2_len, term1_2, &term1, 16);
    double term2_1[16];
    int    term2_1_len = o.Gen_Diff(l__sy_tx_len, l__sy_tx, n__qy_tv_len, n__qy_tv, term2_1);
    double term2_2_p[16], *term2_2 = term2_2_p;
    int    term2_2_len = o.Gen_Sum_With_PreAlloc(term2_1_len, term2_1, o__qx_sv_len, o__qx_sv, &term2_2, 16);
    double term2_p[16], *term2 = term2_p;
    int    term2_len = o.Gen_Product_With_PreAlloc(2, ah, term2_2_len, term2_2, &term2, 16);
    double term3_1[16];
    int    term3_1_len = o.Gen_Diff(l__ry_tw_len, l__ry_tw, m__qy_tv_len, m__qy_tv, term3_1);
    double term3_2_p[16], *term3_2 = term3_2_p;
    int    term3_2_len = o.Gen_Sum_With_PreAlloc(term3_1_len, term3_1, o__qw_rv_len, o__qw_rv, &term3_2, 16);
    double term3_p[16], *term3 = term3_p;
    int    term3_len = o.Gen_Product_With_PreAlloc(2, ai, term3_2_len, term3_2, &term3, 16);
    double term4_1[16];
    int    term4_1_len = o.Gen_Diff(l__rx_sw_len, l__rx_sw, m__qx_sv_len, m__qx_sv, term4_1);
    double term4_2_p[16], *term4_2 = term4_2_p;
    int    term4_2_len = o.Gen_Sum_With_PreAlloc(term4_1_len, term4_1, n__qw_rv_len, n__qw_rv, &term4_2, 16);
    double term4_p[16], *term4 = term4_p;
    int    term4_len = o.Gen_Product_With_PreAlloc(2, aj, term4_2_len, term4_2, &term4, 16);
    double term5_1[16];
    int    term5_1_len = o.Gen_Diff(m__sy_tx_len, m__sy_tx, n__ry_tw_len, n__ry_tw, term5_1);
    double term5_2_p[16], *term5_2 = term5_2_p;
    int    term5_2_len = o.Gen_Sum_With_PreAlloc(term5_1_len, term5_1, o__rx_sw_len, o__rx_sw, &term5_2, 16);
    double term5_p[16], *term5 = term5_p;
    int    term5_len = o.Gen_Product_With_PreAlloc(2, bf, term5_2_len, term5_2, &term5, 16);
    double term6_1[16];
    int    term6_1_len = o.Gen_Diff(k__sy_tx_len, k__sy_tx, n__py_tu_len, n__py_tu, term6_1);
    double term6_2_p[16], *term6_2 = term6_2_p;
    int    term6_2_len = o.Gen_Sum_With_PreAlloc(term6_1_len, term6_1, o__px_su_len, o__px_su, &term6_2, 16);
    double term6_p[16], *term6 = term6_p;
    int    term6_len = o.Gen_Product_With_PreAlloc(2, bh, term6_2_len, term6_2, &term6, 16);
    double term7_1[16];
    int    term7_1_len = o.Gen_Diff(k__ry_tw_len, k__ry_tw, m__py_tu_len, m__py_tu, term7_1);
    double term7_2_p[16], *term7_2 = term7_2_p;
    int    term7_2_len = o.Gen_Sum_With_PreAlloc(term7_1_len, term7_1, o__pw_ru_len, o__pw_ru, &term7_2, 16);
    double term7_p[16], *term7 = term7_p;
    int    term7_len = o.Gen_Product_With_PreAlloc(2, bi, term7_2_len, term7_2, &term7, 16);
    double term8_1[16];
    int    term8_1_len = o.Gen_Diff(k__rx_sw_len, k__rx_sw, m__px_su_len, m__px_su, term8_1);
    double term8_2_p[16], *term8_2 = term8_2_p;
    int    term8_2_len = o.Gen_Sum_With_PreAlloc(term8_1_len, term8_1, n__pw_ru_len, n__pw_ru, &term8_2, 16);
    double term8_p[16], *term8 = term8_p;
    int    term8_len = o.Gen_Product_With_PreAlloc(2, bj, term8_2_len, term8_2, &term8, 16);
    double term9_1[16];
    int    term9_1_len = o.Gen_Diff(l__sy_tx_len, l__sy_tx, n__qy_tv_len, n__qy_tv, term9_1);
    double term9_2_p[16], *term9_2 = term9_2_p;
    int    term9_2_len = o.Gen_Sum_With_PreAlloc(term9_1_len, term9_1, o__qx_sv_len, o__qx_sv, &term9_2, 16);
    double term9_p[16], *term9 = term9_p;
    int    term9_len = o.Gen_Product_With_PreAlloc(2, cf, term9_2_len, term9_2, &term9, 16);
    double term10_1[16];
    int    term10_1_len = o.Gen_Diff(k__sy_tx_len, k__sy_tx, n__py_tu_len, n__py_tu, term10_1);
    double term10_2_p[16], *term10_2 = term10_2_p;
    int    term10_2_len = o.Gen_Sum_With_PreAlloc(term10_1_len, term10_1, o__px_su_len, o__px_su, &term10_2, 16);
    double term10_p[16], *term10 = term10_p;
    int    term10_len = o.Gen_Product_With_PreAlloc(2, cg, term10_2_len, term10_2, &term10, 16);
    double term11_1[16];
    int    term11_1_len = o.Gen_Diff(k__qy_tv_len, k__qy_tv, l__py_tu_len, l__py_tu, term11_1);
    double term11_2_p[16], *term11_2 = term11_2_p;
    int    term11_2_len = o.Gen_Sum_With_PreAlloc(term11_1_len, term11_1, o__pv_qu_len, o__pv_qu, &term11_2, 16);
    double term11_p[16], *term11 = term11_p;
    int    term11_len = o.Gen_Product_With_PreAlloc(2, ci, term11_2_len, term11_2, &term11, 16);
    double term12_1[16];
    int    term12_1_len = o.Gen_Diff(k__qx_sv_len, k__qx_sv, l__px_su_len, l__px_su, term12_1);
    double term12_2_p[16], *term12_2 = term12_2_p;
    int    term12_2_len = o.Gen_Sum_With_PreAlloc(term12_1_len, term12_1, n__pv_qu_len, n__pv_qu, &term12_2, 16);
    double term12_p[16], *term12 = term12_p;
    int    term12_len = o.Gen_Product_With_PreAlloc(2, cj, term12_2_len, term12_2, &term12, 16);
    double term13_1[16];
    int    term13_1_len = o.Gen_Diff(l__ry_tw_len, l__ry_tw, m__qy_tv_len, m__qy_tv, term13_1);
    double term13_2_p[16], *term13_2 = term13_2_p;
    int    term13_2_len = o.Gen_Sum_With_PreAlloc(term13_1_len, term13_1, o__qw_rv_len, o__qw_rv, &term13_2, 16);
    double term13_p[16], *term13 = term13_p;
    int    term13_len = o.Gen_Product_With_PreAlloc(2, df, term13_2_len, term13_2, &term13, 16);
    double term14_1[16];
    int    term14_1_len = o.Gen_Diff(k__ry_tw_len, k__ry_tw, m__py_tu_len, m__py_tu, term14_1);
    double term14_2_p[16], *term14_2 = term14_2_p;
    int    term14_2_len = o.Gen_Sum_With_PreAlloc(term14_1_len, term14_1, o__pw_ru_len, o__pw_ru, &term14_2, 16);
    double term14_p[16], *term14 = term14_p;
    int    term14_len = o.Gen_Product_With_PreAlloc(2, dg, term14_2_len, term14_2, &term14, 16);
    double term15_1[16];
    int    term15_1_len = o.Gen_Diff(k__qy_tv_len, k__qy_tv, l__py_tu_len, l__py_tu, term15_1);
    double term15_2_p[16], *term15_2 = term15_2_p;
    int    term15_2_len = o.Gen_Sum_With_PreAlloc(term15_1_len, term15_1, o__pv_qu_len, o__pv_qu, &term15_2, 16);
    double term15_p[16], *term15 = term15_p;
    int    term15_len = o.Gen_Product_With_PreAlloc(2, dh, term15_2_len, term15_2, &term15, 16);
    double term16_1[16];
    int    term16_1_len = o.Gen_Diff(k__qw_rv_len, k__qw_rv, l__pw_ru_len, l__pw_ru, term16_1);
    double term16_2_p[16], *term16_2 = term16_2_p;
    int    term16_2_len = o.Gen_Sum_With_PreAlloc(term16_1_len, term16_1, m__pv_qu_len, m__pv_qu, &term16_2, 16);
    double term16_p[16], *term16 = term16_p;
    int    term16_len = o.Gen_Product_With_PreAlloc(2, dj, term16_2_len, term16_2, &term16, 16);
    double term17_1[16];
    int    term17_1_len = o.Gen_Diff(l__rx_sw_len, l__rx_sw, m__qx_sv_len, m__qx_sv, term17_1);
    double term17_2_p[16], *term17_2 = term17_2_p;
    int    term17_2_len = o.Gen_Sum_With_PreAlloc(term17_1_len, term17_1, n__qw_rv_len, n__qw_rv, &term17_2, 16);
    double term17_p[16], *term17 = term17_p;
    int    term17_len = o.Gen_Product_With_PreAlloc(2, ef, term17_2_len, term17_2, &term17, 16);
    double term18_1[16];
    int    term18_1_len = o.Gen_Diff(k__rx_sw_len, k__rx_sw, m__px_su_len, m__px_su, term18_1);
    double term18_2_p[16], *term18_2 = term18_2_p;
    int    term18_2_len = o.Gen_Sum_With_PreAlloc(term18_1_len, term18_1, n__pw_ru_len, n__pw_ru, &term18_2, 16);
    double term18_p[16], *term18 = term18_p;
    int    term18_len = o.Gen_Product_With_PreAlloc(2, eg, term18_2_len, term18_2, &term18, 16);
    double term19_1[16];
    int    term19_1_len = o.Gen_Diff(k__qx_sv_len, k__qx_sv, l__px_su_len, l__px_su, term19_1);
    double term19_2_p[16], *term19_2 = term19_2_p;
    int    term19_2_len = o.Gen_Sum_With_PreAlloc(term19_1_len, term19_1, n__pv_qu_len, n__pv_qu, &term19_2, 16);
    double term19_p[16], *term19 = term19_p;
    int    term19_len = o.Gen_Product_With_PreAlloc(2, eh, term19_2_len, term19_2, &term19, 16);
    double term20_1[16];
    int    term20_1_len = o.Gen_Diff(k__qw_rv_len, k__qw_rv, l__pw_ru_len, l__pw_ru, term20_1);
    double term20_2_p[16], *term20_2 = term20_2_p;
    int    term20_2_len = o.Gen_Sum_With_PreAlloc(term20_1_len, term20_1, m__pv_qu_len, m__pv_qu, &term20_2, 16);
    double term20_p[16], *term20 = term20_p;
    int    term20_len = o.Gen_Product_With_PreAlloc(2, ei, term20_2_len, term20_2, &term20, 16);
    double result1_p[16], *result1 = result1_p;
    int    result1_len = o.Gen_Diff_With_PreAlloc(term1_len, term1, term2_len, term2, &result1, 16);
    double result2_p[16], *result2 = result2_p;
    int    result2_len = o.Gen_Sum_With_PreAlloc(result1_len, result1, term3_len, term3, &result2, 16);
    double result3_p[16], *result3 = result3_p;
    int    result3_len = o.Gen_Diff_With_PreAlloc(result2_len, result2, term4_len, term4, &result3, 16);
    double result4_p[16], *result4 = result4_p;
    int    result4_len = o.Gen_Diff_With_PreAlloc(result3_len, result3, term5_len, term5, &result4, 16);
    double result5_p[16], *result5 = result5_p;
    int    result5_len = o.Gen_Sum_With_PreAlloc(result4_len, result4, term6_len, term6, &result5, 16);
    double result6_p[16], *result6 = result6_p;
    int    result6_len = o.Gen_Diff_With_PreAlloc(result5_len, result5, term7_len, term7, &result6, 16);
    double result7_p[16], *result7 = result7_p;
    int    result7_len = o.Gen_Sum_With_PreAlloc(result6_len, result6, term8_len, term8, &result7, 16);
    double result8_p[16], *result8 = result8_p;
    int    result8_len = o.Gen_Sum_With_PreAlloc(result7_len, result7, term9_len, term9, &result8, 16);
    double result9_p[16], *result9 = result9_p;
    int    result9_len = o.Gen_Diff_With_PreAlloc(result8_len, result8, term10_len, term10, &result9, 16);
    double result10_p[16], *result10 = result10_p;
    int    result10_len = o.Gen_Sum_With_PreAlloc(result9_len, result9, term11_len, term11, &result10, 16);
    double result11_p[16], *result11 = result11_p;
    int    result11_len = o.Gen_Diff_With_PreAlloc(result10_len, result10, term12_len, term12, &result11, 16);
    double result12_p[16], *result12 = result12_p;
    int    result12_len = o.Gen_Diff_With_PreAlloc(result11_len, result11, term13_len, term13, &result12, 16);
    double result13_p[16], *result13 = result13_p;
    int    result13_len = o.Gen_Sum_With_PreAlloc(result12_len, result12, term14_len, term14, &result13, 16);
    double result14_p[16], *result14 = result14_p;
    int    result14_len = o.Gen_Diff_With_PreAlloc(result13_len, result13, term15_len, term15, &result14, 16);
    double result15_p[16], *result15 = result15_p;
    int    result15_len = o.Gen_Sum_With_PreAlloc(result14_len, result14, term16_len, term16, &result15, 16);
    double result16_p[16], *result16 = result16_p;
    int    result16_len = o.Gen_Sum_With_PreAlloc(result15_len, result15, term17_len, term17, &result16, 16);
    double result17_p[16], *result17 = result17_p;
    int    result17_len = o.Gen_Diff_With_PreAlloc(result16_len, result16, term18_len, term18, &result17, 16);
    double result18_p[16], *result18 = result18_p;
    int    result18_len = o.Gen_Sum_With_PreAlloc(result17_len, result17, term19_len, term19, &result18, 16);
    double result_p[16], *result = result_p;
    int    result_len = o.Gen_Diff_With_PreAlloc(result18_len, result18, term20_len, term20, &result, 16);

    double return_value = result[result_len - 1];
    if (result_p != result) free(result);
    if (result18_p != result18) free(result18);
    if (result17_p != result17) free(result17);
    if (result16_p != result16) free(result16);
    if (result15_p != result15) free(result15);
    if (result14_p != result14) free(result14);
    if (result13_p != result13) free(result13);
    if (result12_p != result12) free(result12);
    if (result11_p != result11) free(result11);
    if (result10_p != result10) free(result10);
    if (result9_p != result9) free(result9);
    if (result8_p != result8) free(result8);
    if (result7_p != result7) free(result7);
    if (result6_p != result6) free(result6);
    if (result5_p != result5) free(result5);
    if (result4_p != result4) free(result4);
    if (result3_p != result3) free(result3);
    if (result2_p != result2) free(result2);
    if (result1_p != result1) free(result1);
    if (term20_p != term20) free(term20);
    if (term20_2_p != term20_2) free(term20_2);
    if (term19_p != term19) free(term19);
    if (term19_2_p != term19_2) free(term19_2);
    if (term18_p != term18) free(term18);
    if (term18_2_p != term18_2) free(term18_2);
    if (term17_p != term17) free(term17);
    if (term17_2_p != term17_2) free(term17_2);
    if (term16_p != term16) free(term16);
    if (term16_2_p != term16_2) free(term16_2);
    if (term15_p != term15) free(term15);
    if (term15_2_p != term15_2) free(term15_2);
    if (term14_p != term14) free(term14);
    if (term14_2_p != term14_2) free(term14_2);
    if (term13_p != term13) free(term13);
    if (term13_2_p != term13_2) free(term13_2);
    if (term12_p != term12) free(term12);
    if (term12_2_p != term12_2) free(term12_2);
    if (term11_p != term11) free(term11);
    if (term11_2_p != term11_2) free(term11_2);
    if (term10_p != term10) free(term10);
    if (term10_2_p != term10_2) free(term10_2);
    if (term9_p != term9) free(term9);
    if (term9_2_p != term9_2) free(term9_2);
    if (term8_p != term8) free(term8);
    if (term8_2_p != term8_2) free(term8_2);
    if (term7_p != term7) free(term7);
    if (term7_2_p != term7_2) free(term7_2);
    if (term6_p != term6) free(term6);
    if (term6_2_p != term6_2) free(term6_2);
    if (term5_p != term5) free(term5);
    if (term5_2_p != term5_2) free(term5_2);
    if (term4_p != term4) free(term4);
    if (term4_2_p != term4_2) free(term4_2);
    if (term3_p != term3) free(term3);
    if (term3_2_p != term3_2) free(term3_2);
    if (term2_p != term2) free(term2);
    if (term2_2_p != term2_2) free(term2_2);
    if (term1_p != term1) free(term1);
    if (term1_2_p != term1_2) free(term1_2);

    if (return_value > 0) return IP_Sign::POSITIVE;
    if (return_value < 0) return IP_Sign::NEGATIVE;
    if (return_value == 0) return IP_Sign::ZERO;
    return IP_Sign::UNDEFINED;
}

int det5(double a_,
         double b_,
         double c_,
         double d_,
         double e_,
         double f_,
         double g_,
         double h_,
         double i_,
         double j_,
         double k_,
         double l_,
         double m_,
         double n_,
         double o_,
         double p_,
         double q_,
         double r_,
         double s_,
         double t_,
         double u_,
         double v_,
         double w_,
         double x_,
         double y_)
{
    int ret;
#ifdef IMPLICIT_PREDICATES_STAGE_STATS
    semi_static_filter_stage++;
#endif
    ret = det5_filtered(a_, b_, c_, d_, e_, f_, g_, h_, i_, j_, k_, l_, m_, n_, o_, p_, q_, r_, s_, t_, u_, v_, w_, x_, y_);
    if (ret != Filtered_Sign::UNCERTAIN) return ret;

#ifdef IMPLICIT_PREDICATES_STAGE_STATS
    interval_arithmetic_stage++;
#endif
    ret = det5_interval(a_, b_, c_, d_, e_, f_, g_, h_, i_, j_, k_, l_, m_, n_, o_, p_, q_, r_, s_, t_, u_, v_, w_, x_, y_);
    if (ret != Filtered_Sign::UNCERTAIN) return ret;

#ifdef IMPLICIT_PREDICATES_STAGE_STATS
    exact_computation_stage++;
#endif
    return det5_exact(a_, b_, c_, d_, e_, f_, g_, h_, i_, j_, k_, l_, m_, n_, o_, p_, q_, r_, s_, t_, u_, v_, w_, x_, y_);
}