// $Id: analyticsystemmodel_gaussianuncertainty.cpp 29495 2008-08-13 12:57:49Z tdelaet $
// Copyright (C) 2002 Klaas Gadeyne <first dot last at gmail dot com>
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation; either version 2.1 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
//

#include "include/analyticsystemmodel_gaussianuncertainty.h"

using namespace Eigen;

namespace bfl_core
{
    // Constructor
    AnalyticSystemModelGaussianUncertainty::AnalyticSystemModelGaussianUncertainty
            (AnalyticConditionalGaussian *Systempdf)
            : SystemModel<Vector3d>(Systempdf)
    {}

    // Destructor
    AnalyticSystemModelGaussianUncertainty::~AnalyticSystemModelGaussianUncertainty()
    {}


    Matrix3d
    AnalyticSystemModelGaussianUncertainty::df_dxGet(const Vector3d &u,
                                                     const Vector3d &x)
    {
        SystemPdfGet()->ConditionalArgumentSet(0, x);
        if (SystemPdfGet()->NumConditionalArgumentsGet() == 2)
            SystemPdfGet()->ConditionalArgumentSet(1, u);
        return dynamic_cast<AnalyticConditionalGaussian *>(SystemPdfGet())->dfGet(0);
    }


    Vector3d
    AnalyticSystemModelGaussianUncertainty::PredictionGet(const Vector3d &u,
                                                          const Vector3d &x)
    {
        SystemPdfGet()->ConditionalArgumentSet(0, x);
        if (SystemPdfGet()->NumConditionalArgumentsGet() == 2)
            SystemPdfGet()->ConditionalArgumentSet(1, u);
        return SystemPdfGet()->ExpectedValueGet();
    }


    Matrix3d
    AnalyticSystemModelGaussianUncertainty::CovarianceGet(const Vector3d &u,
                                                          const Vector3d &x)
    {
        SystemPdfGet()->ConditionalArgumentSet(0, x);
        if (SystemPdfGet()->NumConditionalArgumentsGet() == 2)
            SystemPdfGet()->ConditionalArgumentSet(1, u);
        return dynamic_cast<AnalyticConditionalGaussian *>(SystemPdfGet())->CovarianceGet();
    }


} // End namespace BFL
